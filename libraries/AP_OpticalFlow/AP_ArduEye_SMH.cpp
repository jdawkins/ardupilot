

#include "AP_ArduEye_SMH.h"

/*********************************************************************/
//	begin
//	Initializes the vision chips for normal operation.  Sets vision
//	chip pins to low outputs, clears chip registers, sets biases and
//	config register.  If no parameters are passed in, default values
//	are used.
/*********************************************************************/
ArduEyeSMH::ArduEyeSMH(AP_HAL::AnalogSource* analog,
								AP_HAL::DigitalSource* resp,
								AP_HAL::DigitalSource* incp,
								AP_HAL::DigitalSource* resv,
								AP_HAL::DigitalSource* incv,
								AP_HAL::DigitalSource* inphi,
								AP_HAL::Scheduler* sched,
								AP_HAL::UARTDriver* console):
	_analog(analog),_resp(resp),_incp(incp),_resv(resv),_incv(incv),_inphi(inphi),_sched(sched),_console(console)
	{

		
	}
void ArduEyeSMH::begin(short vref,short nbias,short aobias,char gain)
{
		
		_resp->write(0);
		_incp->write(0);
		_resv->write(0);
		_incv->write(0);
		_inphi->write(0);
  //clear all chip register values
  clearValues();

 // _console->printf_P(PSTR("vref %i\n"),vref);
  //_console->printf_P(PSTR("nbias %i\n"),nbias);  
  //_console->printf_P(PSTR("aobias %i\n"),aobias);
 
  
  //set up biases
  setBiases(vref,nbias,aobias);

  short config=gain+(16);
   
  //turn chip on with config value
  setPointerValue(SMH_SYS_CONFIG,config);
  
   //_console->printf_P(PSTR("config %i\n"),config);
  //if amp is used, set useAmp variable


}

/*********************************************************************/
//	setPointer
//	Sets the pointer system register to the desired value
/*********************************************************************/

void ArduEyeSMH::setPointer(char ptr)
{
  // clear pointer
  pulseResP();//SMH1_ResP_Pulse; // macro

  // increment pointer to desired value
  for (short i=0; i!=ptr; ++i) 
    pulseIncP();//SMH1_IncP_Pulse; // macro
}

/*********************************************************************/
//	setValue
//	Sets the value of the current register
/*********************************************************************/

void ArduEyeSMH::setValue(short val) 
{
  // clear pointer
  pulseResV();//SMH1_ResV_Pulse; // macro

  // increment pointer
  for (short i=0; i!=val; ++i) 
    pulseIncV();//SMH1_IncV_Pulse; // macro
}

/*********************************************************************/
//	incValue
//	Sets the pointer system register to the desired value.  Value is
//	not reset so the current value must be taken into account
/*********************************************************************/

void ArduEyeSMH::incValue(short val) {
  for (short i=0; i<val; ++i) //increment pointer
    pulseIncV();//SMH1_IncV_Pulse;
}

/*********************************************************************/
//	pulseInphi
//	Operates the amplifier.  Sets inphi pin high, delays to allow
//	value time to settle, and then brings it low.
/*********************************************************************/
void ArduEyeSMH::pulseResP(){
	_resp->pulse();
}
 void ArduEyeSMH::pulseIncP(){
	_incp->pulse();

}  
void ArduEyeSMH::pulseResV(){
	_resv->pulse();

}
void ArduEyeSMH::pulseIncV(){
	_incv->pulse();

}

 void ArduEyeSMH::pulseInphi(char delay){
 //_console->printf_P(PSTR("Inphi\n"));
  _inphi->write(1);//SMH1_InPhi_SetHigh;
  //_sched->delay_microseconds(delay);
  _inphi->write(0);//SMH1_InPhi_SetLow;
}

/*********************************************************************/
//	setPointerValue
//	Sets the pointer to a register and sets the value of that        
//	register
/*********************************************************************/

void ArduEyeSMH::setPointerValue(char ptr,short val)
{
	setPointer(ptr);	//set pointer to register
      setValue(val);	//set value of that register
}

/*********************************************************************/
//	clearValues
//	Resets the value of all registers to zero
/*********************************************************************/

void ArduEyeSMH::clearValues(void)
{
    for (char i=0; i!=8; ++i)
    	setPointerValue(i,0);	//set each register to zero
}

/*********************************************************************/
//	setVREF
//	Sets the VREF register value (0-63)
/*********************************************************************/

void  ArduEyeSMH::setVREF(short vref)
{
  setPointerValue(SMH_SYS_VREF,vref);
}

/*********************************************************************/
//	setNBIAS
//	Sets the NBIAS register value (0-63)
/*********************************************************************/

void  ArduEyeSMH::setNBIAS(short nbias)
{
  setPointerValue(SMH_SYS_NBIAS,nbias);
}

/*********************************************************************/
//	setAOBIAS
//	Sets the AOBIAS register value (0-63)
/*********************************************************************/

void  ArduEyeSMH::setAOBIAS(short aobias)
{
  setPointerValue(SMH_SYS_AOBIAS,aobias);
}

/*********************************************************************/
//	setBiasesVdd
//	Sets biases based on chip voltage
/*********************************************************************/

void ArduEyeSMH::setBiasesVdd(char vddType)
{
  
  // determine biases. Only one option for now.
  switch (vddType) 
  {
    case SMH1_VDD_5V0:	//chip receives 5V
    default:
      setPointerValue(SMH_SYS_NBIAS,SMH_NBIAS_5V0);
	setPointerValue(SMH_SYS_AOBIAS,SMH_AOBIAS_5V0);
	setPointerValue(SMH_SYS_VREF,SMH_VREF_5V0);
    break;
  }
}

/*********************************************************************/
//	setBiases
//	Sets all three biases
/*********************************************************************/

void ArduEyeSMH::setBiases(short vref,short nbias,short aobias)
{
   	setPointerValue(SMH_SYS_NBIAS,nbias);
	setPointerValue(SMH_SYS_AOBIAS,aobias);
	setPointerValue(SMH_SYS_VREF,vref);
}

/*********************************************************************/
//	setConfig
//	Sets configuration register
//	cvdda:  (1) connect vdda, always should be connected
//	selamp: (0) bypasses amplifier, (1) connects it
//	gain: amplifier gain 1-7
//	EXAMPLE 1: To configure the chip to bypass the amplifier:
//	setConfig(0,0,1);
//	EXAMPLE 2: To use the amplifier and set the gain to 4:
//	setConfig(4,1,1);
/*********************************************************************/

void ArduEyeSMH::setConfig(char gain, char selamp, char cvdda) 
{
   short config=gain+(selamp*8)+(cvdda*16);	//form register value

   if(selamp==1)	//if selamp is 1, set useAmp variable to 1
     useAmp=1;
   else 
     useAmp=0;
  
   // Note that config will have the following form binary form:
   // 000csggg where c=cvdda, s=selamp, ggg=gain (3 bits)
   // Note that there is no overflow detection in the input values.
   setPointerValue(SMH_SYS_CONFIG,config);
}

/*********************************************************************/
//	setAmpGain
//	A friendlier version of setConfig.  If amplifier gain is set to 
//	zero, amplifier is bypassed.  Otherwise the appropriate amplifier
//	gain (range 1-7) is set.
/*********************************************************************/

void ArduEyeSMH::setAmpGain(char gain)
{
   short config;

   if((gain>0)&&(gain<8))	//if gain is a proper value, connect amp
   {
    config=gain+8+16;	//gain+(selamp=1)+(cvdda=1)
    useAmp=1;	//using amplifier
   }
   else	//if gain is zero or outside range, bypass amp
   {
    config=16;	//(cvdda=1)
    useAmp=0;	//bypassing amplifier
   }

   setPointerValue(SMH_SYS_CONFIG,config);	//set config register
}



/*********************************************************************/
//	setBinning
//	Configures binning in the focal plane using the VSW and HSW
//	system registers. The super pixels are aligned with the top left 
//	of the image, e.g. "offset downsampling" is not used. This 
//	function is for the Stonyman chip only. 
//	VARIABLES:
//	hbin: set to 1, 2, 4, or 8 to bin horizontally by that amount
//	vbin: set to 1, 2, 4, or 8 to bin vertically by that amount
/*********************************************************************/

void ArduEyeSMH::setBinning(short hbin,short vbin)
{
   short hsw,vsw;

   switch (hbin) //horizontal binning
   {
    case 2:		//downsample by 2
      hsw = 0xAA;
      break;
    case 4:		//downsample by 4
      hsw = 0xEE;
      break;
    case 8:		//downsample by 8
      hsw = 0xFE;
      break;
    default:	//no binning
      hsw = 0x00;
   }

   switch (vbin) 	//vertical binning
   {
    case 2:		//downsample by 2
      vsw = 0xAA;
      break;
    case 4:		//downsample by 4
      vsw = 0xEE;
      break;
    case 8:		//downsample by 8
      vsw = 0xFE;
      break;
    default:	//no binning
      vsw = 0x00;
    }

  //set switching registers
  setPointerValue(SMH_SYS_HSW,hsw);
  setPointerValue(SMH_SYS_VSW,vsw);
}

/*********************************************************************/
//	calcMask
//	Expose the vision chip to uniform texture (such as a white piece
//	of paper placed over the optics).  Take an image using the 
//	getImage function.  Pass the short "img" array and the "size"
//	number of pixels, along with a unsigned char "mask" array to hold
//	the FPN mask and mask_base for the FPN mask base.  Function will
//	populate the mask array and mask_base variable with the FPN mask,
//	which can then be used with the applMask function. 
/*********************************************************************/

void ArduEyeSMH::calcMask(short *img, short size, unsigned char *mask,short *mask_base)
{
 	*mask_base = 10000; // e.g. "high"

      for (int i=0; i<size; ++i)
        if (img[i]<(*mask_base))	//find the min value for mask_base
          *mask_base = img[i];

      // generate calibration mask
      for (int i=0; i<size; ++i)
        mask[i] = img[i] - *mask_base;	//subtract min value for mask
}


void ArduEyeSMH::calcMask(float *img, short size, float *mask,float *mask_base)
{
 	*mask_base = 10000; // e.g. "high"

      for (int i=0; i<size; ++i)
        if (img[i]<(*mask_base))	//find the min value for mask_base
          *mask_base = img[i];

      // generate calibration mask
      for (int i=0; i<size; ++i)
        mask[i] = img[i] - *mask_base;	//subtract min value for mask
}
/*********************************************************************/
//	applyMask
//	given the "mask" and "mask_base" variables calculated in        
//	calcMask, and a current image, this function will subtract the
//	mask to provide a calibrated image.
/*********************************************************************/

void ArduEyeSMH::applyMask(short *img, short size, unsigned char *mask, short mask_base)
{
	 // Subtract calibration mask
  	 for (int i=0; i<size;++i) 
	{
    		img[i] -= mask_base+mask[i];  //subtract FPN mask
    		img[i]=-img[i];          //negate image so it displays properly
 	}
}

void ArduEyeSMH::applyMask(float *img, short size, float *mask, float mask_base)
{
	 // Subtract calibration mask
  	 for (int i=0; i<size;++i) 
	{
    		img[i] -= mask_base+mask[i];  //subtract FPN mask
    		img[i]=-img[i];          //negate image so it displays properly
 	}
}
/*********************************************************************/
//	getImage
//	This function acquires a box section of a Stonyman or Hawksbill 
//	and saves to image array img.  Note that images are read out in 
//	raster manner (e.g. row wise) and stored as such in a 1D array. 
//	In this case the pointer img points to the output array. 
//
//	VARIABLES: 
//	img (output): pointer to image array, an array of signed shorts
//	rowstart: first row to acquire
//	numrows: number of rows to acquire
//	rowskip: skipping between rows (useful if binning is used)
//	colstart: first column to acquire
//	numcols: number of columns to acquire
//	colskip: skipping between columns
//	ADCType: which ADC to use, defined ADC_TYPES
//	anain (0,1,2,3): which analog input to use
//	
//	EXAMPLES:
//	getImage(img,16,8,1,24,8,1,SMH1_ADCTYPE_ONBOARD,0): 
//	Grab an 8x8 window of pixels at raw resolution starting at row 
//	16, column 24, from chip using onboard ADC at input 0
//	getImage(img,0,14,8,0,14,8,SMH1_ADCTYPE_MCP3201,2): 
//	Grab entire Stonyman chip when using
//	8x8 binning. Grab from input 2.
/*********************************************************************/

void ArduEyeSMH::getImage(short*img, unsigned char rowstart, unsigned char numrows, unsigned char rowskip, unsigned char colstart, unsigned char numcols, unsigned char colskip) 
{
 //_console->printf_P(PSTR("Get New Image\n"));
  //float *pimg = img; // pointer to output image array
  int idx = 0;
  double val;
  unsigned char chigh,clow;
  unsigned char row,col;
  		
  setPointerValue(SMH_SYS_ROWSEL,rowstart);

   _console->printf_P(PSTR("Img = [ "));
  // Loop through all rows
  for (row=0; row<numrows; ++row) {
    
    // Go to first column
    setPointerValue(SMH_SYS_COLSEL,colstart);
     // _console->printf_P(PSTR("%f"),val);
    // Loop through all columns
    for (col=0; col<numcols; ++col) {
      
      // get pixel value from ADC

      val = _analog->read_latest();//analogRead(anain); // acquire pixel
	 //val = val +1;
		//short val = (_analog->read_raw());//analogRead(anain); // acquire pixel
    	_console->printf_P(PSTR("%f"),val);
		_console->printf_P(PSTR(" "));
		
	//*pimg = val; // store pixel	
      img[idx] = (short)val; // store pixel
	  
      //img++;
	  idx++;
	  //pimg++; // advance pointer
	  
      incValue(colskip); // go to next column
	  //_sched->delay_microseconds(1);
    }
    setPointer(SMH_SYS_ROWSEL);
	//_sched->delay_microseconds(1);
    incValue(rowskip); // go to next row
	//_sched->delay_microseconds(1);
	_console->printf_P(PSTR("\n"));
  }
    _console->printf_P(PSTR("];\n"));
}


void ArduEyeSMH::dispImage(short *img, unsigned char rowstart, unsigned char numrows, unsigned char rowskip, unsigned char colstart, unsigned char numcols, unsigned char colskip) 
{
 //_console->printf_P(PSTR("Get New Image\n"));
  //float *pimg = img; // pointer to output image array
  //short val;
  int i=0;
  unsigned char chigh,clow;
  unsigned char row,col;
  		
  //setPointerValue(SMH_SYS_ROWSEL,rowstart);
  _console->printf_P(PSTR("Img = [ "));
  // Loop through all rows
  for (row=0; row<numrows; ++row) {
    // Loop through all columns
    for (col=0; col<numcols; ++col) {
   
	   _console->printf_P(PSTR("%i"),img[i]);
       _console->printf_P(PSTR(" "));
      i++; // advance pointer
      //incValue(colskip); // go to next column
    }

	_console->printf_P(PSTR("\n"));
  }
	_console->printf_P(PSTR("];\n"));
}



/*********************************************************************/
//	findMax
//	Searches over a block section of a Stonyman or Hawksbill chip
//	to find the brightest pixel. This function is intended to be used 
//	for things like finding the location of a pinhole in response to 
//	a bright light.
//
//	VARIABLES: 
//	rowstart: first row to search
//	numrows: number of rows to search
//	rowskip: skipping between rows (useful if binning is used)
//	colstart: first column to search
//	numcols: number of columns to search
//	colskip: skipping between columns
//	ADCType: which ADC to use, defined ADC_TYPES
//	anain (0,1,2,3): which analog input to use
//	rowwinner: (output) pointer to variable to write row of brightest 
//	pixel
//	colwinner: (output) pointer to variable to write column of 
//	brightest pixel
//
//	EXAMPLE:
//	FindMaxSlow(8,104,1,8,104,1,SMH1_ADCTYPE_ONBOARD,0,&rowwinner,
//	&colwinner): 
//	Search rows 8...104 and columns 8...104 for brightest pixel, with 
//	onboard ADC, chip 0
/*********************************************************************/

void ArduEyeSMH::findMax(unsigned char rowstart, unsigned char numrows, unsigned char rowskip, unsigned char colstart, unsigned char numcols,unsigned char colskip, char anain,unsigned char *max_row, unsigned char *max_col)
{
  unsigned short maxval=5000,minval=0,val;
  unsigned char row,col,bestrow,bestcol;
  unsigned char chigh,clow;
  

     setAnalogInput(anain);	//set Arduino analog input

  // Go to first row
  setPointerValue(SMH_SYS_ROWSEL,rowstart);

  // Loop through all rows
  for (row=0; row<numrows; ++row) {

    // Go to first column
    setPointerValue(SMH_SYS_COLSEL,colstart);

    // Loop through all columns
    for (col=0; col<numcols; ++col) {

      // settling delay
	  _sched->delay_microseconds(1);
      //delayMicroseconds(1);

      // pulse amplifier if needed
	if (useAmp) 
        pulseInphi(2);
      
      // get data value
	  _sched->delay_microseconds(1);
      //delayMicroseconds(1);
      
      // get pixel from ADC

       val = _analog->voltage_average();//analogRead(anain); // acquire pixel
	   
	if(useAmp)	//amplifier is inverted
	{
		if (val>minval) 	//find max val (bright)
		{
       	 bestrow=row;
       	 bestcol=col;
      	 minval=val;
      	}
	}
	else		//unamplified
	{
      	if (val<maxval) 	//find min val (bright)
		{
        	 bestrow=row;
        	 bestcol=col;
        	 maxval=val;
      	}
	}

      incValue(colskip); // go to next column
    }
    setPointer(SMH_SYS_ROWSEL);
    incValue(rowskip); // go to next row
  }

  // Optionally we can comment out these next three items
  //Serial.print("bestrow = "); Serial.println((short)bestrow);
  //Serial.print("bestcol = "); Serial.println((short)bestcol);
  //Serial.print("maxval = "); Serial.println((short)maxval);

  *max_row = bestrow;
  *max_col = bestcol;
}
