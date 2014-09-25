#pragma once

#include "Vector.h"

struct IMUData
{
	Vector3 EulierAngles;
};

extern QueueHandle_t TheIMUDataQueue;

/// Process data from IMU and push it to TheIMUDataQueue using IMUData.
void vTaskIMUProcessor (void *pvParameters);
