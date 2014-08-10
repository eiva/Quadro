/**
 * Flight Controller based on nRF2400.
 * All side libraries provided by respected autors: see headers.
 *
 * Pins:
 *
 */

#include <Wire.h>
#include <SPI.h>
#include <Servo.h>
#include <EEPROM.h>
#include <SD.h>

#define bool boolean

#include "Mirf.h"
#include "nRF24L01.h"
#include "MirfHardwareSpiDriver.h"
#include "L3G4200D.h"
#include "I2Cdev.h"
#include "ADXL345.h"
#include "Complimentary.h"
#include "Kalman.h"
#include "EepromReader.h"
#include "PidObject.h"
#include "LedInfo.h"
#include "Motors.h"
#include "RadioLink.h"
#include "Stabilizer.h"
#include "RcAxisLimits.h"
#include "BlackBox.h"
#include "Controller.h"


// Quadro controller
Controller TheController;
//Stabilizer TheStab;

void setup(){
  Serial.begin(9600);
  
  TheController.Init();
  //TheStab.Init();
}

void loop(){
    TheController.Update();
	/*TheStab.Update();
	const float stabYaw = TheStab.Yaw;
    const float stabPitch = TheStab.Pitch;
    const float stabRoll = TheStab.Roll;
    Serial.print("YPR ");
    Serial.print(stabYaw);
    Serial.print(" ");
    Serial.print(stabPitch);
    Serial.print(" ");
    Serial.println(stabRoll);*/
}
