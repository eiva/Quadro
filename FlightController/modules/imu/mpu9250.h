#pragma once

class Port;
class SpiInterface;

class Mpu9250{
	SpiInterface* _spiInterface;
	Port* _ncsPort;
public:
	Mpu9250(SpiInterface* spiInterface, Port* ncsPort);
	bool Check();
	void Read(uint8_t ReadBuf[14]);
private:
	uint8_t ReadReg( uint8_t ReadAddr );

	void WriteReg( uint8_t WriteAddr, uint8_t WriteData );

	void ReadRegs( uint8_t ReadAddr, uint8_t *ReadBuf, uint8_t Bytes );
};
