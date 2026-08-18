#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "lwip/sockets.h"
#include "flight_types.hpp"
#include <cstdarg>
#include <cstring>
#include <atomic>

#define LOG_UDP_PORT     5556
#define LOG_BUFFER_SIZE  256
#define LOG_QUEUE_SIZE   32

static const char *TAG = "LOG_SERVER";

struct LogMessage {
    char text[LOG_BUFFER_SIZE];
    int len;
};

static int s_log_sock = -1;
static struct sockaddr_in s_log_client = {};
static std::atomic<bool> s_client_known{false};
static QueueHandle_t s_log_queue = nullptr;
static vprintf_like_t s_original_vprintf = nullptr;

// Vprintf-Hook: nur in Queue schieben, NICHT senden
static int log_vprintf(const char *fmt, va_list args) {
    va_list copy;
    va_copy(copy, args);

    // Originaler UART-Output
    int ret = s_original_vprintf(fmt, args);

    // Nur queuen, wenn Client da ist und Queue existiert
    if (s_client_known.load(std::memory_order_relaxed) && s_log_queue != nullptr) {
        LogMessage msg;
        msg.len = vsnprintf(msg.text, sizeof(msg.text), fmt, copy);
        if (msg.len > 0) {
            if (msg.len >= (int)sizeof(msg.text)) msg.len = sizeof(msg.text) - 1;
            // Non-blocking send to queue, drop on overflow
            xQueueSend(s_log_queue, &msg, 0);
        }
    }

    va_end(copy);
    return ret;
}

// Separate Task, die aus der Queue sendet
static void log_sender_task(void *arg) {
    LogMessage msg;
    while (true) {
        if (xQueueReceive(s_log_queue, &msg, portMAX_DELAY) == pdTRUE) {
            if (s_client_known.load() && s_log_sock >= 0) {
                sendto(s_log_sock, msg.text, msg.len, 0,
                       (struct sockaddr *)&s_log_client, sizeof(s_log_client));
            }
        }
    }
}

void log_server_task(void *pvParameters) {
    xEventGroupWaitBits(s_startup_event_group, WIFI_READY_BIT,
                        pdFALSE, pdTRUE, portMAX_DELAY);

    s_log_sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (s_log_sock < 0) {
        ESP_LOGE(TAG, "Failed to create socket: %d", errno);
        vTaskDelete(nullptr);
        return;
    }

    struct sockaddr_in local = {};
    local.sin_family      = AF_INET;
    local.sin_port        = htons(LOG_UDP_PORT);
    local.sin_addr.s_addr = htonl(INADDR_ANY);

    if (bind(s_log_sock, (struct sockaddr *)&local, sizeof(local)) < 0) {
        ESP_LOGE(TAG, "Failed to bind socket: %d", errno);
        close(s_log_sock);
        vTaskDelete(nullptr);
        return;
    }

    // Queue erstellen
    s_log_queue = xQueueCreate(LOG_QUEUE_SIZE, sizeof(LogMessage));
    if (!s_log_queue) {
        ESP_LOGE(TAG, "Failed to create log queue");
        close(s_log_sock);
        vTaskDelete(nullptr);
        return;
    }

    // Sender-Task starten
    xTaskCreate(log_sender_task, "log_sender", 4096, nullptr, 3, nullptr);

    // Vprintf-Hook installieren
    s_original_vprintf = esp_log_set_vprintf(log_vprintf);
    ESP_LOGI(TAG, "Listening on UDP port %d, waiting for subscriber...", LOG_UDP_PORT);

    char rx[64];
    struct sockaddr_in sender = {};
    socklen_t sender_len = sizeof(sender);

    while (true) {
        int len = recvfrom(s_log_sock, rx, sizeof(rx), 0,
                           (struct sockaddr *)&sender, &sender_len);
        if (len > 0) {
            s_log_client = sender;
            s_client_known.store(true, std::memory_order_release);

            // ESP_LOG NICHT direkt nach dem subscribe, sondern direkt antworten
            const char *ack = "LOG_STREAM_START\n";
            sendto(s_log_sock, ack, strlen(ack), 0,
                   (struct sockaddr *)&s_log_client, sizeof(s_log_client));
        }
    }
}