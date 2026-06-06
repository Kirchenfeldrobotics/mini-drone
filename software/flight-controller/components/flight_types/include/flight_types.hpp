enum BlinkPattern {
    LED_BLINKING, 
    LED_WAITING,
}; 

static EventGroupHandle_t s_startup_event_group; 
#define WIFI_READY_BIT       BIT0
#define UDP_SERVER_READY_BIT BIT1
#define MPU_READY_BIT        BIT2
#define PID_READY_BIT        BIT3
#define DSHOT_READY_BIT      BIT4
#define MOTOR_CLEARANCE_BIT  BIT5
#define MOTOR_ARMED_BIT      BIT6
#define MOTOR_READY_BIT      BIT7