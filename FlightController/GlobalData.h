#pragma once
class GlobalData
{
public:
	GlobalData()
	: BootMilliseconds(0)
	, AttQ0(0), AttQ1(0), AttQ2(0), AttQ3(0)
	{

	}
public:

	uint32_t BootMilliseconds; // Milliseconds from start.

	float AttQ0, AttQ1, AttQ2, AttQ3;
};

extern GlobalData TheGlobalData;
