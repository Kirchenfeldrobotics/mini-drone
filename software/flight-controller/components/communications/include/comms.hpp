#include "freertos/semphr.h"
#include "pid.hpp" 
#include <atomic>
#include <iostream>
#include <thread>

static SemaphoreHandle_t s_target_angles_mutex; 
static TargetDroneAngles s_target_angles; 

static SemaphoreHandle_t s_target_base_throttle_mutex; 
static std::atomic<int> s_target_base_throttle;  