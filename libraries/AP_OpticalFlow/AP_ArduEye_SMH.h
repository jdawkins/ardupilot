/*********************************************************************/
/*********************************************************************/
//	ArduEye_SMH_v1.h
//	ArduEye Library for the Stonyman/Hawksbill Centeye Vision Chips
//	
//	Basic functions to operate Stonyman/Hawksbill with ArduEye boards
//	Supports all Arduino boards that use the ATMega 8/168/328/2560
//	NOTE: ATMega 2560 SPI for external ADC is not supported.
//
//	Working revision started July 9, 2012
//
/*********************************************************************/
/*********************************************************************/

/*
===============================================================================
Copyright (c) 2012 Centeye, Inc. 
All rights reserved.

Redistribution and use in source and binary forms, with or without 
modification, are permitted provided that the following conditions are met:

    Redistributions of source code must retain the above copyright notice, 
    this list of conditions and the following disclaimer.
    
    Redistributions in binary form must reproduce the above copyright notice, 
    this list of conditions and the following disclaimer in the documentation 
    and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY CENTEYE, INC. ``AS IS'' AND ANY EXPRESS OR 
IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF 
MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO 
EVENT SHALL CENTEYE, INC. OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, 
INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, 
BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, 
DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY 
OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, 
EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

The views and conclusions contained in the software and documentation are 
those of the authors and should not be interpreted as representing official 
policies, either expressed or implied, of Centeye, Inc.
===============================================================================
*/

#ifndef _ARDUEYE_SMH_H_INCLUDED
#define _ARDUEYE_SMH_H_INCLUDED

#include <AP_HAL_AVR.h>

#define RESP_PIN 56
#define INCP_PIN 57
#define RESV_PIN 58
#define INCV_PIN 59
#define INPHI_PIN 60
#define ANALOG_PIN 7

//SMH System Registers

#define SMH_SYS_COLSEL 0	//select column
#define SMH_SYS_ROWSEL 1	//select row
#define SMH_SYS_VSW 2		//vertical switching
#define SMH_SYS_HSW 3		//horizontal switching
#define SMH_SYS_CONFIG 5	//configuration register
#define SMH_SYS_VREF 4		//voltage reference
#define SMH_SYS_NBIAS 6		//nbias
#define SMH_SYS_AOBIAS 7	//analog out bias

/*********************************************************************/
//default values

// Supply voltage types
// Notation: AVB is A.B volts. e.g. 5V0 is 5V, 3V3 is 3.3V, etc.
#define SMH1_VDD_5V0  1

#define SMH_VREF_5V0 55		//vref for 5 volts
#define SMH_NBIAS_5V0 55	//nbias for 5 volts
#define SMH_AOBIAS_5V0 55	//aobias for 5 volts

#define SMH_GAIN_DEFAULT 0	//no amp gain
#define SMH_SELAMP_DEFAULT 0	//amp bypassed


/*********************************************************************/
/*********************************************************************/
/*********************************************************************/
/*********************************************************************/
//	ArduEyeSMHClass
/*********************************************************************/
/*********************************************************************/

class ArduEyeSMH 
{

private:

  //indicates whether amplifier is in use	
  char useAmp;
  AP_HAL::AnalogSource* _analog;
  AP_HAL::DigitalSource* _resp;
  AP_HAL::DigitalSource* _incp;
  AP_HAL::DigitalSource* _resv;
  AP_HAL::DigitalSource* _incv;
  AP_HAL::DigitalSource* _inphi;
  AP_HAL::Scheduler* _sched;
  AP_HAL::UARTDriver* _console;

public:

	ArduEyeSMH(AP_HAL::AnalogSource* analog,
				AP_HAL::DigitalSource* resp,
				AP_HAL::DigitalSource* incp,
				AP_HAL::DigitalSource* resv, 
				AP_HAL::DigitalSource* incv, 
				AP_HAL::DigitalSource* inphi,
				AP_HAL::Scheduler* sched,
				AP_HAL::UARTDriver* console);

/*********************************************************************/
// Initialize the vision chip for image readout
  
  void begin(short vref=SMH_VREF_5V0,short nbias=SMH_NBIAS_5V0,short aobias=SMH_AOBIAS_5V0,char gain=SMH_GAIN_DEFAULT); 

/*********************************************************************/
// Chip Register and Value Manipulation

  //set pointer on chip
  void setPointer(char ptr);

  //set value of current pointer
  void setValue(short val);

  //increment value of current pointer
  void incValue(short val);

  //pulse INPHI to operate amplifier
  void pulseInphi(char delay); 

  //functions to pulse lines
  void pulseResP();
  void pulseIncP();  
  void pulseResV();
  void pulseIncV();
  /*void pulseInphi(){
  _inphi->write(1);
  _sched->delay_microseconds(2);
  _inphi->write(0);
  }*/
  //clear all register values
  void clearValues(void);

  //set pointer to register and set value for that register
  void setPointerValue(char ptr,short val);

  //set configuration register on chip
  void setConfig(char gain, char selamp,char cvdda=1);

  //select amp and set amp gain
  void setAmpGain(char gain);

  //set analog input to Arduino for onboard ADC
  void setAnalogInput(char analogInput);

  //set external ADC input
  void setADCInput(char ADCInput,char state);

  //set hsw and vsw registers to bin on-chip
  void setBinning(short hbin,short vbin);

/*********************************************************************/
// Bias functions

  //set individual bias values
  void setVREF(short vref);
  void setNBIAS(short nbias);
  void setAOBIAS(short aobias);
 
  //set biases based on Vdd
  void setBiasesVdd(char vddType);

  //set all bias values
  void setBiases(short vref,short nbias,short aobias);


/*********************************************************************/
// Image Functions

  //given an image, returns a fixed-pattern noise mask and mask_base
  void calcMask(short *img, short size, unsigned char *mask, short *mask_base);
  void calcMask(float *img, short size, float *mask, float *mask_base);
  
  //applies pre-calculated FPN mask to an image
  void applyMask(short *img, short size, unsigned char *mask, short mask_base);
  void applyMask(float *img, short size, float *mask, float mask_base);
  
  //gets an image from the vision chip
  void getImage(short *img, unsigned char rowstart, unsigned char numrows, unsigned char rowskip, unsigned char colstart, unsigned 	char numcols, unsigned char colskip);
  void dispImage(short *img, unsigned char rowstart, unsigned char numrows, unsigned char rowskip, unsigned char colstart, unsigned 	char numcols, unsigned char colskip);


  //takes an image and returns the maximum value row and col
  void findMax(unsigned char rowstart, unsigned char numrows, unsigned char rowskip, unsigned char colstart, unsigned char numcols, unsigned char colskip, char anain,unsigned char *max_row, unsigned char *max_col);

};

//external definition of ArduEyeSMH class instance
//extern ArduEyeSMHClass ArduEyeSMH;

#endif
