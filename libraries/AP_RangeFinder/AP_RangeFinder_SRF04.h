/************************************************************************************************************
 * DistanceSRF04.h - Arduino library for retrieving data from the SRF04 Distance sensor                     *
 * Copyright 2011-2012 Jeroen Doggen (jeroendoggen@gmail.com)                                               *
 * More info: http://www.robot-electronics.co.uk/files/SRF04.pde                                            *
 ************************************************************************************************************
 * Version History:                                                                                         *
 *  Version 0.1: getDistanceCentimeter, blocking                                                            *
 *                                                                                                          *
 * Roadmap:                                                                                                 *
 *  Version 0.2: add filtering (low pass filter?)                                                           *
 *  Version 0.3: getDistanceCentimeter, interrupt based                                                     *                                                                                          *
 ************************************************************************************************************
 * This library is free software; you can redistribute it and/or                                            *
 * modify it under the terms of the GNU Lesser General Public                                               *
 * License as published by the Free Software Foundation; either                                             *
 * version 2.1 of the License, or (at your option) any later version.                                       *
 *                                                                                                          *
 * This library is distributed in the hope that it will be useful,                                          *
 * but WITHOUT ANY WARRANTY; without even the implied warranty of                                           *
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU                                        *
 * Lesser General Public License for more details.                                                          *
 *                                                                                                          *
 * You should have received a copy of the GNU Lesser General Public                                         *
 * License along with this library; if not, write to the Free Software                                      *
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA                               *
 ***********************************************************************************************************/

#ifndef AP_RangeFinder_SRF04_h
#define AP_RangeFinder_SRF04_h

#include <AP_HAL_AVR.h>
/*#if defined(ARDUINO) && ARDUINO >= 100
  #include "Arduino.h"
#else
  #include "WProgram.h"
  #include <pins_arduino.h>
#endif*/
#define AP_RANGEFINDER_SRF04 4
#define AP_RANGEFINDER_SRF04_SCALER 0.512
#define AP_RANGEFINDER_SRF04_MIN_DISTANCE 30
#define AP_RANGEFINDER_SRF04_MAX_DISTANCE 500

#include "RangeFinder.h"
#include <AP_HAL_AVR.h>

class AP_RangeFinder_SRF04
{
	public:
		//AP_RangeFinder_SRF04(const AP_HAL::HAL hal);
		//AP_RangeFinder_SRF04(const AP_HAL::GPIO *gpio);
		AP_RangeFinder_SRF04(AP_HAL::DigitalSource *trigger,AP_HAL::DigitalSource *echo, AP_HAL::Scheduler* sched);

		void begin();
		void begin(int echoPin, int trigPin);
		
		int getDistanceTime();
		int getDistanceCentimeter();
		int getDistanceInch();

		bool isCloser(int threshold);
		bool isFarther(int threshold);

		void setAveraging(int avg);   

		int16_t convert_raw_to_distance(int16_t raw_value) {
        return raw_value;
		}	
		
		int16_t read();
		
		    // maximum measurable distance: in cm
		int16_t  max_distance;
		// minimum measurable distance: in cm
		int16_t  min_distance;

	private:
		
		//const AP_HAL_AVR::AVRGPIO* _gpio;
		//AP_HAL_AVR::AVRGPIO* _echo;
		// AP_HAL_AVR::HAL_AVR_APM2 _hal;
		//AP_HAL::AnalogSource*
		AP_HAL::DigitalSource* _trigger;
		AP_HAL::DigitalSource* _echo;
		AP_HAL::Scheduler* _sched;
		int _trigPin;
		int _echoPin;
		int _average;
		long _duration;
};
#endif
