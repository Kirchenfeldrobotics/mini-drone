
static QueueHandle_t s_pid_data_queue = nullptr; 

struct TargetDroneAngles {
    float pitch, roll, yaw; 
}; 

struct PID {
    float pitch, roll, yaw; 
}; 