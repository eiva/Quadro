#include "stm32f4xx_conf.h"
#include "Port.h"
#include "SpiInterface.h"
#include "mpu9250.h"
#include <string.h>

/*====================================================================================================*/
/*====================================================================================================*/
/*
|     |      ACCELEROMETER      |        GYROSCOPE        |
| LPF | BandW | Delay  | Sample | BandW | Delay  | Sample |
+-----+-------+--------+--------+-------+--------+--------+
|  0  | 260Hz |    0ms |  1kHz  | 256Hz | 0.98ms |  8kHz  |
|  1  | 184Hz |  2.0ms |  1kHz  | 188Hz |  1.9ms |  1kHz  |
|  2  |  94Hz |  3.0ms |  1kHz  |  98Hz |  2.8ms |  1kHz  |
|  3  |  44Hz |  4.9ms |  1kHz  |  42Hz |  4.8ms |  1kHz  |
|  4  |  21Hz |  8.5ms |  1kHz  |  20Hz |  8.3ms |  1kHz  |
|  5  |  10Hz | 13.8ms |  1kHz  |  10Hz | 13.4ms |  1kHz  |
|  6  |   5Hz | 19.0ms |  1kHz  |   5Hz | 18.6ms |  1kHz  |
|  7  | -- Reserved -- |  1kHz  | -- Reserved -- |  8kHz  |
*/
//typedef enum {
//  MPU_LPS_256Hz   = 0x00,
//  MPU_LPS_188Hz   = 0x01,
//  MPU_LPS_98Hz    = 0x02,
//  MPU_LPS_42Hz    = 0x03,
//  MPU_LPS_20Hz    = 0x04,
//  MPU_LPS_10Hz    = 0x05,
//  MPU_LPS_5Hz     = 0x06,
//  MPU_LPS_Disable = 0x07,
//} MPU_LPF_TypeDef;
typedef enum {
  MPU_GyrFS_250dps  = 0x00,
  MPU_GyrFS_500dps  = 0x08,
  MPU_GyrFS_1000dps = 0x10,
  MPU_GyrFS_2000dps = 0x18
} MPU_GyrFS_TypeDef;
typedef enum {
  MPU_AccFS_2g  = 0x00,
  MPU_AccFS_4g  = 0x08,
  MPU_AccFS_8g  = 0x10,
  MPU_AccFS_16g = 0x18
} MPU_AccFS_TypeDef;

//typedef struct {
//  MPU_LPF_TypeDef MPU_LowPassFilter;
//  MPU_GyrFS_TypeDef MPU_Gyr_FullScale;
//  MPU_AccFS_TypeDef MPU_Acc_FullScale;
//} MPU_InitTypeDef;

/* ---- Sensitivity --------------------------------------------------------- */

#define MPU9250A_2g       ((float)0.000061035156f) // 0.000061035156 g/LSB
#define MPU9250A_4g       ((float)0.000122070312f) // 0.000122070312 g/LSB
#define MPU9250A_8g       ((float)0.000244140625f) // 0.000244140625 g/LSB
#define MPU9250A_16g      ((float)0.000488281250f) // 0.000488281250 g/LSB

#define MPU9250G_250dps   ((float)0.007633587786f) // 0.007633587786 dps/LSB
#define MPU9250G_500dps   ((float)0.015267175572f) // 0.015267175572 dps/LSB
#define MPU9250G_1000dps  ((float)0.030487804878f) // 0.030487804878 dps/LSB
#define MPU9250G_2000dps  ((float)0.060975609756f) // 0.060975609756 dps/LSB

#define MPU9250M_4800uT   ((float)0.6f)            // 0.6 uT/LSB

#define MPU9250T_85degC   ((float)0.002995177763f) // 0.002995177763 degC/LSB

/* ---- MPU6500 Reg In MPU9250 ---------------------------------------------- */

#define MPU6500_I2C_ADDR            ((uint8_t)0xD0)
#define MPU6500_Device_ID           ((uint8_t)0x71)  // In MPU9250

#define MPU6500_SELF_TEST_XG        ((uint8_t)0x00)
#define MPU6500_SELF_TEST_YG        ((uint8_t)0x01)
#define MPU6500_SELF_TEST_ZG        ((uint8_t)0x02)
#define MPU6500_SELF_TEST_XA        ((uint8_t)0x0D)
#define MPU6500_SELF_TEST_YA        ((uint8_t)0x0E)
#define MPU6500_SELF_TEST_ZA        ((uint8_t)0x0F)
#define MPU6500_XG_OFFSET_H         ((uint8_t)0x13)
#define MPU6500_XG_OFFSET_L         ((uint8_t)0x14)
#define MPU6500_YG_OFFSET_H         ((uint8_t)0x15)
#define MPU6500_YG_OFFSET_L         ((uint8_t)0x16)
#define MPU6500_ZG_OFFSET_H         ((uint8_t)0x17)
#define MPU6500_ZG_OFFSET_L         ((uint8_t)0x18)
#define MPU6500_SMPLRT_DIV          ((uint8_t)0x19)
#define MPU6500_CONFIG              ((uint8_t)0x1A)
#define MPU6500_GYRO_CONFIG         ((uint8_t)0x1B)
#define MPU6500_ACCEL_CONFIG        ((uint8_t)0x1C)
#define MPU6500_ACCEL_CONFIG_2      ((uint8_t)0x1D)
#define MPU6500_LP_ACCEL_ODR        ((uint8_t)0x1E)
#define MPU6500_MOT_THR             ((uint8_t)0x1F)
#define MPU6500_FIFO_EN             ((uint8_t)0x23)
#define MPU6500_I2C_MST_CTRL        ((uint8_t)0x24)
#define MPU6500_I2C_SLV0_ADDR       ((uint8_t)0x25)
#define MPU6500_I2C_SLV0_REG        ((uint8_t)0x26)
#define MPU6500_I2C_SLV0_CTRL       ((uint8_t)0x27)
#define MPU6500_I2C_SLV1_ADDR       ((uint8_t)0x28)
#define MPU6500_I2C_SLV1_REG        ((uint8_t)0x29)
#define MPU6500_I2C_SLV1_CTRL       ((uint8_t)0x2A)
#define MPU6500_I2C_SLV2_ADDR       ((uint8_t)0x2B)
#define MPU6500_I2C_SLV2_REG        ((uint8_t)0x2C)
#define MPU6500_I2C_SLV2_CTRL       ((uint8_t)0x2D)
#define MPU6500_I2C_SLV3_ADDR       ((uint8_t)0x2E)
#define MPU6500_I2C_SLV3_REG        ((uint8_t)0x2F)
#define MPU6500_I2C_SLV3_CTRL       ((uint8_t)0x30)
#define MPU6500_I2C_SLV4_ADDR       ((uint8_t)0x31)
#define MPU6500_I2C_SLV4_REG        ((uint8_t)0x32)
#define MPU6500_I2C_SLV4_DO         ((uint8_t)0x33)
#define MPU6500_I2C_SLV4_CTRL       ((uint8_t)0x34)
#define MPU6500_I2C_SLV4_DI         ((uint8_t)0x35)
#define MPU6500_I2C_MST_STATUS      ((uint8_t)0x36)
#define MPU6500_INT_PIN_CFG         ((uint8_t)0x37)
#define MPU6500_INT_ENABLE          ((uint8_t)0x38)
#define MPU6500_INT_STATUS          ((uint8_t)0x3A)
#define MPU6500_ACCEL_XOUT_H        ((uint8_t)0x3B)
#define MPU6500_ACCEL_XOUT_L        ((uint8_t)0x3C)
#define MPU6500_ACCEL_YOUT_H        ((uint8_t)0x3D)
#define MPU6500_ACCEL_YOUT_L        ((uint8_t)0x3E)
#define MPU6500_ACCEL_ZOUT_H        ((uint8_t)0x3F)
#define MPU6500_ACCEL_ZOUT_L        ((uint8_t)0x40)
#define MPU6500_TEMP_OUT_H          ((uint8_t)0x41)
#define MPU6500_TEMP_OUT_L          ((uint8_t)0x42)
#define MPU6500_GYRO_XOUT_H         ((uint8_t)0x43)
#define MPU6500_GYRO_XOUT_L         ((uint8_t)0x44)
#define MPU6500_GYRO_YOUT_H         ((uint8_t)0x45)
#define MPU6500_GYRO_YOUT_L         ((uint8_t)0x46)
#define MPU6500_GYRO_ZOUT_H         ((uint8_t)0x47)
#define MPU6500_GYRO_ZOUT_L         ((uint8_t)0x48)
#define MPU6500_EXT_SENS_DATA_00    ((uint8_t)0x49)
#define MPU6500_EXT_SENS_DATA_01    ((uint8_t)0x4A)
#define MPU6500_EXT_SENS_DATA_02    ((uint8_t)0x4B)
#define MPU6500_EXT_SENS_DATA_03    ((uint8_t)0x4C)
#define MPU6500_EXT_SENS_DATA_04    ((uint8_t)0x4D)
#define MPU6500_EXT_SENS_DATA_05    ((uint8_t)0x4E)
#define MPU6500_EXT_SENS_DATA_06    ((uint8_t)0x4F)
#define MPU6500_EXT_SENS_DATA_07    ((uint8_t)0x50)
#define MPU6500_EXT_SENS_DATA_08    ((uint8_t)0x51)
#define MPU6500_EXT_SENS_DATA_09    ((uint8_t)0x52)
#define MPU6500_EXT_SENS_DATA_10    ((uint8_t)0x53)
#define MPU6500_EXT_SENS_DATA_11    ((uint8_t)0x54)
#define MPU6500_EXT_SENS_DATA_12    ((uint8_t)0x55)
#define MPU6500_EXT_SENS_DATA_13    ((uint8_t)0x56)
#define MPU6500_EXT_SENS_DATA_14    ((uint8_t)0x57)
#define MPU6500_EXT_SENS_DATA_15    ((uint8_t)0x58)
#define MPU6500_EXT_SENS_DATA_16    ((uint8_t)0x59)
#define MPU6500_EXT_SENS_DATA_17    ((uint8_t)0x5A)
#define MPU6500_EXT_SENS_DATA_18    ((uint8_t)0x5B)
#define MPU6500_EXT_SENS_DATA_19    ((uint8_t)0x5C)
#define MPU6500_EXT_SENS_DATA_20    ((uint8_t)0x5D)
#define MPU6500_EXT_SENS_DATA_21    ((uint8_t)0x5E)
#define MPU6500_EXT_SENS_DATA_22    ((uint8_t)0x5F)
#define MPU6500_EXT_SENS_DATA_23    ((uint8_t)0x60)
#define MPU6500_I2C_SLV0_DO         ((uint8_t)0x63)
#define MPU6500_I2C_SLV1_DO         ((uint8_t)0x64)
#define MPU6500_I2C_SLV2_DO         ((uint8_t)0x65)
#define MPU6500_I2C_SLV3_DO         ((uint8_t)0x66)
#define MPU6500_I2C_MST_DELAY_CTRL  ((uint8_t)0x67)
#define MPU6500_SIGNAL_PATH_RESET   ((uint8_t)0x68)
#define MPU6500_MOT_DETECT_CTRL     ((uint8_t)0x69)
#define MPU6500_USER_CTRL           ((uint8_t)0x6A)
#define MPU6500_PWR_MGMT_1          ((uint8_t)0x6B)
#define MPU6500_PWR_MGMT_2          ((uint8_t)0x6C)
#define MPU6500_FIFO_COUNTH         ((uint8_t)0x72)
#define MPU6500_FIFO_COUNTL         ((uint8_t)0x73)
#define MPU6500_FIFO_R_W            ((uint8_t)0x74)
#define MPU6500_WHO_AM_I            ((uint8_t)0x75)	// ID = 0x71 In MPU9250
#define MPU6500_XA_OFFSET_H         ((uint8_t)0x77)
#define MPU6500_XA_OFFSET_L         ((uint8_t)0x78)
#define MPU6500_YA_OFFSET_H         ((uint8_t)0x7A)
#define MPU6500_YA_OFFSET_L         ((uint8_t)0x7B)
#define MPU6500_ZA_OFFSET_H         ((uint8_t)0x7D)
#define MPU6500_ZA_OFFSET_L         ((uint8_t)0x7E)

/*====================================================================================================*/
Mpu9250::Mpu9250(SpiInterface* spiInterface, Port* ncsPort):
	_spiInterface(spiInterface),_ncsPort(ncsPort)
{
	_ncsPort->High();

	const int MPU9250_InitRegNum = 10;
	uint8_t MPU6500_Init_Data[MPU9250_InitRegNum][2] = {
	      {0x80, MPU6500_PWR_MGMT_1},     // Reset Device
	      {0x01, MPU6500_PWR_MGMT_1},     // Clock Source
	      {0x00, MPU6500_PWR_MGMT_2},     // Enable Acc & Gyro
	      {0x07, MPU6500_CONFIG},         //
	      {MPU_GyrFS_500dps, MPU6500_GYRO_CONFIG},    // +-500dps
	      {MPU_AccFS_4g, MPU6500_ACCEL_CONFIG},   // +-4G
	      {0x00, MPU6500_ACCEL_CONFIG_2}, // Set Acc Data Rates
	      {0x30, MPU6500_INT_PIN_CFG},    //
	      {0x40, MPU6500_I2C_MST_CTRL},   // I2C Speed 348 kHz
	      {0x20, MPU6500_USER_CTRL},      // Enable AUX

	    };

	  for(uint8_t i=0; i<MPU9250_InitRegNum; i++) {
		int j;
		for(j = 0; j< 10000; ++j);
	    WriteReg(MPU6500_Init_Data[i][1], MPU6500_Init_Data[i][0]);
	  }
}

bool Mpu9250::Check(){
    /* MPU6500 */
	uint8_t DeviceID = ReadReg(MPU6500_WHO_AM_I);
	if(DeviceID != MPU6500_Device_ID)
		return false;

	return true;
}

void Mpu9250::Read( uint8_t ReadBuf[14])
{
  // Read 14 bytes from MPU6500_ACCEL_XOUT_H.
  ReadRegs(MPU6500_ACCEL_XOUT_H, ReadBuf, 14);
}

uint8_t Mpu9250::ReadReg( uint8_t ReadAddr )
{
	  _ncsPort->Low();//IMU_CSM = 0;
	  //SPI_RW(SPIx, 0x80 | ReadAddr);
	  //*ReadData = SPI_RW(SPIx, 0xFF);
	  _spiInterface->ReadWrite(0x80 | ReadAddr);
	  uint8_t data = _spiInterface->ReadWrite(0xFF);
	  _ncsPort->High();//IMU_CSM = 1;
	  return data;
	}

	void Mpu9250::WriteReg( uint8_t WriteAddr, uint8_t WriteData )
	{
	  _ncsPort->Low();//IMU_CSM = 0;
	  _spiInterface->ReadWrite(WriteAddr);
	  _spiInterface->ReadWrite(WriteData);
	  _ncsPort->High();//IMU_CSM = 1;
	}

	void Mpu9250::ReadRegs( uint8_t ReadAddr, uint8_t *ReadBuf, uint8_t Bytes )
	{
	  _ncsPort->Low();//IMU_CSM = 0;
	  _spiInterface->ReadWrite(0x80 | ReadAddr);
	  for(uint8_t i=0; i<Bytes; i++)
	    ReadBuf[i] = _spiInterface->ReadWrite(0xFF);
	  _ncsPort->High();//IMU_CSM = 1;
	}
/*====================================================================================================*/
/*====================================================================================================*/
