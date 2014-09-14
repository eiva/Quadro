#pragma once

class Mpu9250{
	SpiInterface& _spiInterface;
	Port& _ncsPort;
public:
	Mpu9250(SpiInterface& spiInterface, Port& ncsPort);
	bool Check();
	void Read(uint8_t ReadBuf[14]);
private:
	uint8_t ReadReg( uint8_t ReadAddr ){
	  _ncsPort.Low();//IMU_CSM = 0;
	  //SPI_RW(SPIx, 0x80 | ReadAddr);
	  //*ReadData = SPI_RW(SPIx, 0xFF);
	  _spiInterface.ReadWrite(0x80 | ReadAddr);
	  uint8_t data = _spiInterface.ReadWrite(0xFF);
	  _ncsPort.High();//IMU_CSM = 1;
	  return data;
	}

	void WriteReg( uint8_t WriteAddr, uint8_t WriteData ){
	  _ncsPort.Low();//IMU_CSM = 0;
	  _spiInterface.ReadWrite(WriteAddr);
	  _spiInterface.ReadWrite(WriteData);
	  _ncsPort.High();//IMU_CSM = 1;
	}

	void ReadRegs( uint8_t ReadAddr, uint8_t *ReadBuf, uint8_t Bytes ){
	  _ncsPort.Low();//IMU_CSM = 0;
	  _spiInterface.ReadWrite(0x80 | ReadAddr);
	  for(uint8_t i=0; i<Bytes; i++)
	    ReadBuf[i] = _spiInterface.ReadWrite(0xFF);
	  _ncsPort.High();//IMU_CSM = 1;
	}
};
