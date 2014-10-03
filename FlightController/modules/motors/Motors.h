#pragma once


class Motors{
	const uint16_t _min;
	const uint16_t _max;
public:
	Motors();
	// Input - values for motor: 0 - 100%.
	void SetRatio(const float m1, const float m2, const float m3, const float m4);
};
