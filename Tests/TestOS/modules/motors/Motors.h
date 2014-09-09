#pragma once


class Motors{
	uint16_t _min;
	uint16_t _max;
public:
	Motors();
	// Input - value from 0 - 1800 = 1/10 degree.
	void SetRatio(const uint16_t m1, const uint16_t m2, const uint16_t m3, const uint16_t m4);
};
