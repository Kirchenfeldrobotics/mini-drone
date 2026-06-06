#pragma once

#include "freertos/semphr.h"
#include "lwip/sockets.h"
#include "lwip/netdb.h"
#include "pid.hpp" 
#include <atomic>
#include <iostream>
#include <thread>

extern SemaphoreHandle_t s_target_angles_mutex; 
extern TargetDroneAngles s_target_angles; 

extern std::atomic<int> s_target_base_throttle;  

struct __attribute__((packed)) ControlPacket {
    uint32_t magic; 
    uint32_t sequence; 
    float target_pitch_angle; 
    float target_roll_angle; 
    float target_yaw_rate; 
    uint16_t base_throttle; 
    uint8_t flags; // BIT0: start, 
    uint8_t _padding; 
}; 

void udp_server_task(void *pvParameters); 