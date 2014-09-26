#pragma once


class Motors{
	const uint16_t _min;
	const uint16_t _max;
public:
	Motors();
	// Input - value from 0 - 100 = 1/10 degree.
	void SetRatio(const float m1, const float m2, const float m3, const float m4);
};
