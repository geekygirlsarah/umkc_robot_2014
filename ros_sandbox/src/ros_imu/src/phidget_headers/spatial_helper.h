/*
Helper methods to transition from Phidget Events to SpatialPVectors.


*/


#ifndef SPATIAL_HELPER_H 
#define SPATIAL_HELPER_H

#include <stdio.h>
#include <deque>

#include <phidget21.h>			//linux
#include <iostream>

namespace spatial	{ 

	//Set up event handlers for spatial.	
	//data rate in milliseconds, must be between 4ms and 1s
	int spatial_setup(CPhidgetSpatialHandle &spatial, std::deque<CPhidgetSpatial_SpatialEventData>* raw, int dataRate );


	//Queue to store data that spatial pushes out.
	typedef std::deque<CPhidgetSpatial_SpatialEventData> PhidgetRawDataQ; 

	//Print out a spatial packet.
	void print(CPhidgetSpatial_SpatialEventData& data);
	CPhidgetSpatial_SpatialEventData* copy(CPhidgetSpatial_SpatialEventData& spatial);

	//Zeroes offset in the Gyro
	void zeroGyro(CPhidgetSpatial_SpatialEventData& data);

}

/*SPATIAL_HELPER_H*/
#endif	
