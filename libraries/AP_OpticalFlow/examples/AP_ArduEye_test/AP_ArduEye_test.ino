/*
 *  Example of AP_OpticalFlow library.
 *  Code by Randy Mackay. DIYDrones.com
 */

#include <AP_Common.h>
#include <AP_Progmem.h>
#include <AP_Param.h>
#include <AP_Math.h>
#include <AP_HAL.h>
#include <AP_HAL_AVR.h>

#include <AP_OpticalFlow.h>

#include <AP_ArduEye_SMH.h>  //Stonyman/Hawksbill vision chip library
//#include <ArduEye_GUI.h>  //ArduEye processing GUI interface
#include <AP_ArduEye_OFO.h>  //Optical Flow support
#include <CYE_Images_v1.h>  //Some image support functions

//==============================================================================
// GLOBAL VARIABLES

//for mega 2560 can handle two 16x16 arrays and FPN mask
//so we set SKIP_PIXELS=4 to downsample by 4
//and START_ROW/COL at row/col 24.  This gives us
//a 16x16 array with superpixels of 4x4.  With the
//start row and col at 24, the 64x64 raw grid is 
//centered on the Stonyman 112x112 raw array.
//#if defined(__AVR_ATmega2560__)  
#define MAX_ROWS 16
#define MAX_COLS 16
#define MAX_PIXELS (MAX_ROWS*MAX_COLS)
#define SKIP_PIXELS 4
#define START_ROW 24
#define START_COL 24 

#define SUBWINDOW_SIZE  8
// SUBWINDOW_NUMPIXLES = number of pixels in the subwindow
#define SUBWINDOW_NUMPIXELS (SUBWINDOW_SIZE*SUBWINDOW_SIZE)
#define WINDOWSIZE 16
#define NUMPIXELS (WINDOWSIZE*WINDOWSIZE)
// recall from note above that image arrays are stored row-size in a 1D array

short last_img[MAX_PIXELS];         //1D image array
short current_img[MAX_PIXELS];
short W1[SUBWINDOW_NUMPIXELS]; // previous frame
short W2[SUBWINDOW_NUMPIXELS]; // current frame

short rowsize=MAX_ROWS;            //maximum rows allowed by memory
short colsize=MAX_COLS;            //maximum cols allowed by memory
short skiprow=SKIP_PIXELS;     //pixels to be skipped during readout because of downsampling
short skipcol=SKIP_PIXELS;     //pixels to be skipped during readout because of downsampling 
short start_row=START_ROW;          //first pixel row to read
short start_col=START_COL;          //first pixel col to read

// Row and column of pinhole
unsigned char mPinhole,nPinhole; 

short chipSelect=0;            //which vision chip to read from

// FPN calibration. To save memory we are placing the FPN calibration
// array in an unsigned char, since the variation between pixels may be 
// expressed with a single byte. Variable mask_base holds an offset value 
// applied to the entire FPN array. Thus the FPN mask for pixel i will be the 
// value mask[i]+mask_base.
unsigned char mask[MAX_PIXELS]; // 16x16 FPN calibration image
short mask_base=0; // FPN calibration image base.


// Command inputs - for receiving commands from user via Serial terminal
char command; // command character
int commandArgument; // argument of command

//a set of vectors to send down
char vectors[2];

char OFType=0;

//optical flow X and Y
short filtered_OFx=0,filtered_OFy=0;
short OFx=0,OFy=0;
short ODOx,ODOy;
bool of_flag = false;
bool iscalibrated = false;
bool first_time = false;

AP_HAL::DigitalSource *resp;
AP_HAL::DigitalSource *incp;
AP_HAL::DigitalSource *resv;
AP_HAL::DigitalSource *incv;
AP_HAL::DigitalSource *inphi;
AP_HAL::AnalogSource *analog;
AP_HAL::Scheduler *sched;
AP_HAL::UARTDriver *console;

const AP_HAL::HAL& hal = AP_HAL_BOARD_DRIVER;

ArduEyeSMH* ardueye;
ArduEyeOFO* optflow;

//AP_OpticalFlow_ADNS3080 optflow;

void setup()
{
    hal.console->println("ArduPilot Mega OpticalFlow library test ver 1.5");

    hal.scheduler->delay(10);

		resp = hal.gpio->channel(56);
		incp = hal.gpio->channel(57);
		resv = hal.gpio->channel(58);
		incv = hal.gpio->channel(59);
		inphi =hal.gpio->channel(60);
		analog = hal.analogin->channel(7);
		
		resp->mode(HAL_GPIO_OUTPUT);
		incp->mode(HAL_GPIO_OUTPUT);
		resv->mode(HAL_GPIO_OUTPUT);
		incv->mode(HAL_GPIO_OUTPUT);
		inphi->mode(HAL_GPIO_OUTPUT);
		sched = hal.scheduler;
		console = hal.console;
  // initialize serial port

 ardueye = new ArduEyeSMH(analog,resp,incp,resv,incv,inphi,sched,console);   
 optflow = new ArduEyeOFO();//initialize ArduEye Stonyman

   //initialize ArduEye Stonyman
  ardueye->begin();
   //set the initial binning on the vision chip
  ardueye->setBinning(skipcol,skiprow);

    hal.scheduler->delay(1000);
}

//
// display menu
//
void display_menu()
{
    hal.console->println();
    hal.console->println("please choose from the following options:");
    hal.console->println("     c - display all config");
	hal.console->println("     f - pefrom calibration");
    hal.console->println("     i - display image");
    hal.console->println("     I - display image continuously");
    hal.console->println("     m - display motion");
    hal.console->println();
}

//
// display config
//
void display_config()
{
   // ardueye->dispImage(mask,sr,row,skiprow,sc,col,skipcol);
    hal.scheduler->delay_microseconds(50);
}

void calibrate_sensor()
{
	hal.console->println("Capturing mask...");
	
	ardueye->getImage(current_img,start_row,rowsize,skiprow,start_col,colsize,skipcol);
	ardueye->calcMask(current_img,MAX_PIXELS,mask,&mask_base);
	iscalibrated = true;

}
// display_image - captures and displays image from flowSensor flowSensor
void display_image()
{
    hal.console->println("image data --------------");
	ardueye->getImage(current_img,start_row,rowsize,skiprow,start_col,colsize,skipcol);
	
	if(iscalibrated){
		ardueye->applyMask(current_img,MAX_PIXELS,mask,mask_base);
	}
    ardueye->dispImage(current_img,start_row,rowsize,skiprow,start_col,colsize,skipcol);

    hal.console->println("-------------------------");
}

// display_image - captures and displays image from flowSensor
void display_image_continuously()
{
    int i;
    hal.console->println("press any key to return to menu");

    //hal.console->flush();

    while( !hal.console->available() ) {
        display_image();
        i=0;
        while( i<20 && !hal.console->available() ) {
            hal.scheduler->delay(100);          // give the viewer a bit of time to catchup
            i++;
        }
    }
}

//
// display motion - show x,y and squal values constantly until user presses a key
//
void display_motion()
{
   
	first_time = true;
    // display instructions on how to exit
    hal.console->println("press x to return to menu..");
    hal.scheduler->delay(10);

	// ACQUIRE A 16x16 IMAGE FROM THE STONYMAN CHIP  
  //ArduEyeSMH.getImage(A,mPinhole-WINDOWSIZE/2,WINDOWSIZE,1,nPinhole-WINDOWSIZE/2,WINDOWSIZE,1,SMH1_ADCTYPE_ONBOARD,0);
  
  // Subtract calibration mask
  //ArduEyeSMH.applyMask(A,NUMPIXELS,C,Cbase);
	ardueye->getImage(current_img,start_row,rowsize,skiprow,start_col,colsize,skipcol);
		if(iscalibrated)
		ardueye->applyMask(current_img,MAX_PIXELS,mask,mask_base);
		
	hal.scheduler->delay(100);
    while (!hal.console->available()) {
	
	
	ardueye->getImage(current_img,start_row,rowsize,skiprow,start_col,colsize,skipcol);
		if(iscalibrated){
			ardueye->applyMask(current_img,MAX_PIXELS,mask,mask_base);
		}
			optflow->IIA_Plus_2D(current_img,last_img,rowsize,colsize,200,&OFx,&OFy);
		   // CYE_ImgShortCopy(W2,W1,SUBWINDOW_NUMPIXELS);
			// Extract middle 8x8 array of pixels from A and store in W2
			//CYE_SubwinShort2D(current_img,W2,WINDOWSIZE,WINDOWSIZE,5,8,5,8);
			// Compute optical flow between W1 and W2 and store in OFx and OFy
			//optflow->IIA_Plus_2D(W1,W2,8,8,200,&OFx,&OFy);
			optflow->LPF(&filtered_OFx,&OFx,0.35);
			optflow->LPF(&filtered_OFy,&OFy,0.35);
    // if "starting up" set OFx and OFy to zero
    if (first_time) {
      OFx=OFy=filtered_OFx=filtered_OFy=ODOx=ODOy=0;
    }
    // Add to accumulations
    ODOx += (int)filtered_OFx;
    ODOy += (int)filtered_OFy;

        // x,y,squal
        hal.console->printf_P(PSTR("%5d %5d %5d %5d\n"),(int)filtered_OFx,(int)filtered_OFy,ODOx,ODOy);
        

        // short delay
		//copy current_img to last_img so two frames are kept
		//for optical flow calculation
		//CYE_ImgShortCopy(current_img,last_img,row*col);
		CYE_ImgShortCopy(current_img,last_img,MAX_PIXELS);
        hal.scheduler->delay(10);
		first_time = false;
    }
}

void loop()
{
    int value;
	//ardueye->getImage(current_img,start_row,rowsize,skiprow,start_col,colsize,skipcol);
	//hal.scheduler->delay(1000);
	//ardueye->dispImage(current_img,sr,row,skiprow,sc,col,skipcol);
    // display menu to user
    display_menu();

    // wait for user to enter something
    while( !hal.console->available() ) {
        hal.scheduler->delay(20);
    }

    // get character from user
    value = hal.console->read();

    switch( value ) {

    case 'c':
        // display all config
        display_config();
        break;

	case 'f':	
		calibrate_sensor();
		break;
		
    case 'i':
        // display image
        display_image();
        break;

    case 'I':
        // display image continuously
        display_image_continuously();
        break;

    case 'm':
        // display motion
        display_motion();
        break;

    default:
        hal.console->println("unrecognised command");
        hal.console->println();
        break;
    }
}

AP_HAL_MAIN();
