#ifdef __cplusplus
 extern "C" {
#endif


#define PI 3.1415926535897932384626433832795
#define HALF_PI 1.5707963267948966192313216916398
#define TWO_PI 6.283185307179586476925286766559
#define DEG_TO_RAD 0.017453292519943295769236907684886
#define RAD_TO_DEG 57.295779513082320876798154814105

#define mapDeg(val, in_Amp, deg_Amp) (((float) val - in_Amp/2.0f) * (deg_Amp / (in_Amp / 2.0f)) * DEG_TO_RAD)

long map(long x, long in_min, long in_max, long out_min, long out_max);

#define min(a,b) ((a)<(b)?(a):(b))
#define max(a,b) ((a)>(b)?(a):(b))
#define abs(x) ((x)>0?(x):-(x))
#define constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))
#define round(x)     ((x)>=0?(long)((x)+0.5):(long)((x)-0.5))
#define radians(deg) ((deg)*DEG_TO_RAD)
#define degrees(rad) ((rad)*RAD_TO_DEG)
#define sq(x) ((x)*(x))

 //������� ��������� ��������
void Delay(volatile uint32_t nTime);

// Time in milliseconds since MCU starts.
uint32_t Millis();

// Time in microseconds since MCU starts.
uint32_t Micros();

#ifdef __cplusplus
}
#endif
