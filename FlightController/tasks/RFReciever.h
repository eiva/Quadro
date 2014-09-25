#pragma once

struct RadioLinkData
{
	float Throttle; // 0 - 100.
	float Yaw, Pitch, Roll; // -100 - +100
};

extern QueueHandle_t TheRadioCommandsQueue;

/// Read data from RadioLink, apply calibration and push it to TheRadioCommandsQueue normalized.
void vTaskRFReciever (void *pvParameters);
