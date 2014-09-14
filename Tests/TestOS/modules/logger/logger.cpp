#include "stm32f4xx_conf.h"
#include "ff_support.h"
#include "sdio_sd.h"
#include "ff.h"
#include "logger.h"
#include "Button.h"
#include "LedInfo.h"

Logger::Logger(LedInfo *info, Button *button):
 _ledInfo(info),
 _button(button),
 _isMounted(false)
{
	_file = new FIL();
	_ledInfo->Y(true);
	_ledInfo->G(true);
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
	_isMounted = true;
	_ledInfo->Y(false);
	_ledInfo->G(false);
}

void Logger::mount()
{

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
	if (_button->GetState())
	{
		if (_isMounted)
		{
			_ledInfo->Y(true);
			f_close(_file);
			f_mount(NULL, "", 0);
			delete _file;
			_isMounted = false;
			_ledInfo->Y(false);
		}
	}

	if (!_isMounted)
	{
		return;
	}

	_ledInfo->Y(true);
	const char* format = "%U %D %D %D %D\r\n";
		f_printf(_file, format,
				data.Timer,
				data.InputThrottle, data.InputYaw, data.InputPitch, data.InputRoll);
	_ledInfo->Y(false);
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
