#include "freertos/semphr.h"
#include "pid.hpp" 

static SemaphoreHandle_t s_target_angels_mutex; 
static TargetDroneAngles s_target_angles; 

static SemaphoreHandle_t s_target_base_throttle_mutex; 
static int s_target_base_throttle; 