/*
 *  AP_RangeFinder_test
 *  Code by DIYDrones.com
 */

// includes
#include <AP_Common.h>
#include <AP_Progmem.h>
#include <AP_Param.h>
#include <AP_HAL.h>
#include <AP_Math.h>
#include <AP_RangeFinder.h>
#include <AP_ADC.h>
#include <AP_ADC_AnalogSource.h>
#include <Filter.h>
#include <AP_Buffer.h>
#include <AP_HAL_AVR.h>

// define Pitot tube's ADC Channel
#define AP_RANGEFINDER_PITOT_TYPE_ADC_CHANNEL 7

////////////////////////////////////////////////////////////////////////////////
// hal.console-> ports
////////////////////////////////////////////////////////////////////////////////

AP_HAL::DigitalSource *trigger;
AP_HAL::DigitalSource *echo;
AP_HAL::Scheduler *sched;


const AP_HAL::HAL& hal = AP_HAL_BOARD_DRIVER;

// declare global instances
//ModeFilterInt16_Size5 mode_filter(2);
//AP_RangeFinder_MaxsonarXL *rf;
AP_RangeFinder_SRF04 *rf;
void setup()
{
    hal.console->println("Range Finder Test v1.1");
    //hal.console->print("Sonar Type: ");
    //hal.console->println(SONAR_TYPE);
     trigger = hal.gpio->channel(54);
     echo = hal.gpio->channel(55);
     sched = hal.scheduler;
     
     trigger->mode(1);
     echo->mode(0);
     rf = new AP_RangeFinder_SRF04(trigger,echo,sched);
    //rf = new AP_RangeFinder_MaxsonarXL(analog_source, &mode_filter);
    //rf->calculate_scaler(SONAR_TYPE, scaling);   // setup scaling for sonar
}

void loop()
{
    
  //trigger->write(0);
  //sched->delay_microseconds(2);
  //trigger->write(1);		
  //sched->delay_microseconds(10);
 // trigger->write(0);
		
//		_duration = _echo->pulseIn(_echoPin, 1,1000);
    hal.console->printf("dist: %d\n",(int)rf->read());
    hal.scheduler->delay(50);
}

AP_HAL_MAIN();
