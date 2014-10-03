#pragma once

extern QueueSetHandle_t TheStabilizerQueueSet;

struct CommanderData
{
	float Yaw, Pitch, Roll; // -100 - 100 - just abstract values.
	float Throttle; // 0 - 100; - percent of power.
};

extern QueueHandle_t TheCommanderDataQueue;

void vTaskCommander (void *pvParameters);
