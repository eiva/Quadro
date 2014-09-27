#pragma once

#include "Vector.h"

struct IMUData
{
	float Yaw, Pitch, Roll;
};

extern QueueHandle_t TheIMUDataQueue;

/// Process data from IMU and push it to TheIMUDataQueue using IMUData.
void vTaskIMUProcessor (void *pvParameters);
