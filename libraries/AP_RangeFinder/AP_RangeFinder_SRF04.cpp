/************************************************************************************************************
 * DistanceSRF04.h - Arduino library for retrieving data from the SFR04 Distance sensor                     *
 * Copyright 2011 Jeroen Doggen (jeroendoggen@gmail.com)                                                    *
 * For more information: variable declaration, changelog,... see DistanceSRF04.h                            *
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

/// <summary>
/// DistanceSRF04.cpp - Library for retrieving data from the GP2Y0A21YK IR Distance sensor.
/// For more information: variable declaration, changelog,... see DistanceSRF04.h
/// </summary>

//#include <Arduino.h>
#include <AP_RangeFinder_SRF04.h>

/// <summary>
/// Constructor
/// </summary>
/*AP_RangeFinder_SRF04::AP_RangeFinder_SRF04(const AP_HAL::GPIO *gpio):_gpio(gpio)
{

}*/
AP_RangeFinder_SRF04::AP_RangeFinder_SRF04(AP_HAL::DigitalSource *trigger,AP_HAL::DigitalSource *echo, AP_HAL::Scheduler* sched):
_trigger(trigger),_echo(echo),_sched(sched)
{
max_distance = 3000;
min_distance = 10;
setAveraging(10);
begin();	
}

/// <summary>
/// Begin function to set default pins
/// </summary>
void AP_RangeFinder_SRF04::begin()
{
	begin (55,54);

}

/// <summary>
/// Begin variables
/// - int trigPin: pin used to activate the sensor
/// - int echoPin: pin used to read the reflection
/// </summary>
void AP_RangeFinder_SRF04::begin(int echoPin, int trigPin)
{
   //_gpio->channel(trigPin+54);
   // _gpio->chanel(echoPin+54);
	_trigPin=trigPin;
	_echoPin=echoPin;
	//pinMode(_trigPin, OUTPUT);
	//pinMode(_echoPin, INPUT);
	setAveraging(1);		      //1: all samples passed to higher level
}
int16_t AP_RangeFinder_SRF04::read(){
	//    _trigger->write(0);
	//	_sched->delay_microseconds(1000);
	//	_trigger->write(1);
	//	_sched->delay_microseconds(1000);		
	getDistanceCentimeter();
}
/// <summary>
/// setAveraging(int avg): Sets how many samples have to be averaged in getDistanceCentimeter, default value is 100.
/// </summary>
void AP_RangeFinder_SRF04::setAveraging(int avg)
{
	_average=avg;
}

/// <summary>
/// getDistanceTime(): Returns the time between transmission and echo receive
/// </summary>
int AP_RangeFinder_SRF04::getDistanceTime()
{
	long sum = 0;
	/*	_trigger->write(0);
		_sched->delay_microseconds(1000);
		_trigger->write(1);
		_sched->delay_microseconds(1000);*/
		
	for (int i=0;i<_average;i++)
	{
  
  
		_trigger->write(0);
		_sched->delay_microseconds(2);
		_trigger->write(1);		
		_sched->delay_microseconds(10);
		_trigger->write(0);
		
		_duration = _echo->pulseIn(55, 1,100000);
		sum=sum+_duration;
	}
	return(int(sum/_average));
}

/// <summary>
/// getDistanceCentimeter(): Returns the distance in centimeters
/// </summary>
int AP_RangeFinder_SRF04::getDistanceCentimeter()
{
	return (getDistanceTime()/29/2);
}

/// <summary>
/// getDistanceInch(): Returns the distance in inches
/// </summary>
int AP_RangeFinder_SRF04::getDistanceInch()
{
	return (getDistanceTime()/74/2);
}

/// <summary>
/// isCloser: check whether the distance to the detected object is smaller than a given threshold
/// </summary>
bool AP_RangeFinder_SRF04::isCloser(int threshold)
{
	if (threshold>getDistanceCentimeter())
	{
		return (true);
	}
	else
	{
		return (false);
	}
}

/// <summary>
/// isFarther: check whether the distance to the detected object is smaller than a given threshold
/// </summary>
bool AP_RangeFinder_SRF04::isFarther(int threshold)
{
	if (threshold<getDistanceCentimeter())
	{
		return true;
	}
	else
	{
		return false;
	}
}


