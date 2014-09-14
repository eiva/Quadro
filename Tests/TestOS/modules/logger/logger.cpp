#include "stm32f4xx_conf.h"
#include "ff_support.h"
#include "sdio_sd.h"
#include "ff.h"
#include "logger.h"

Logger::Logger()
{
	SD_LowLevel_Init();
	if (SD_Detect() != SD_PRESENT)
	{
		_isDetected = false;
		return;
	}
	FRESULT FRes;
	FATFS Fatfs;

	FRes = f_mount(&Fatfs, "", 0);
	if (FRes != RES_OK) {_isDetected = false; return;};

	FRes = f_open(_file, "log.txt", FA_WRITE | FA_OPEN_ALWAYS);
	if (FRes != RES_OK) {_isDetected = false; return;};

	_isDetected = true;
	/*FRes = f_write(_file, buffer, 512*2, &index);
	if (FRes != RES_OK) {_isDetected = false; return;};


		FRes = f_close(&File);
		if (FRes != RES_OK) {_isDetected = false; return;};


		f_mount(NULL, "", 0);*/
}

inline int16_t ftoi(const float input)
{
	return int(input*1000.0f);
}

void Logger::Log(const LogData& data)
{
	if (!_isDetected)
	{
		return;
	}
	const char* format = "%U %D %D %D %D\n";
		f_printf(_file, format,
				data.Timer,
				data.InputThrottle, data.InputYaw, data.InputPitch, data.InputRoll);
	/*
	const char* format = "%U %D %D %D %D %D %D %D %D %D %D %D %D %D %D %D %D %D %U %U %U %U\n";
	f_printf(_file, format,
			data.Timer,
			data.InputThrottle, data.InputYaw, data.InputPitch, data.InputRoll,
			ftoi(data.GyroX), ftoi(data.GyroY), ftoi(data.GyroZ),
			ftoi(data.AccelX), ftoi(data.AccelY), ftoi(data.AccelZ),
			ftoi(data.MagX), ftoi(data.MagY), ftoi(data.MagZ),
			ftoi(data.Alt),
			ftoi(data.Yaw), ftoi(data.Roll), ftoi(data.Pitch),
			data.Motor1, data.Motor2, data.Motor3, data.Motor4
			);
			*/
}
