#include "spatial_helper.h"
#include "phidget_handlers.h"
#include "../config.h"			//GYRO_OFFSET (bias)
#include <stdlib.h>			//for malloc
using namespace std;

void spatial::zeroGyro(CPhidgetSpatial_SpatialEventData& data){
	for(int i =0; i< 3; i++)	{
		data.angularRate[i] = data.angularRate[i] - GYRO_OFFSET_R[i];	
	}
}

CPhidgetSpatial_SpatialEventData* spatial::copy(CPhidgetSpatial_SpatialEventData &other )	{

	CPhidgetSpatial_SpatialEventData* dataHolder = (CPhidgetSpatial_SpatialEventData*)malloc(sizeof(CPhidgetSpatial_SpatialEventData));

	//copying timestamp
	dataHolder->timestamp.seconds = other.timestamp.seconds;
	dataHolder->timestamp.microseconds = other.timestamp.microseconds;

	//copying stuff
	for(int i =0; i < 3; i++)	{
		dataHolder->acceleration[i] = other.acceleration[i];
		dataHolder->angularRate[i] = other.angularRate[i];
		dataHolder->magneticField[i] = other.magneticField[i];
	}

	return dataHolder;
}
	
void spatial::print(CPhidgetSpatial_SpatialEventData& data)	{
	
	
	cout  << "time: " << data.timestamp.seconds << "s " <<
		data.timestamp.microseconds << "microsecs"<< "\t";
	//cout << "Acc " << data.acceleration[0] << " " <<  data.acceleration[1] << " " <<  data.acceleration[2]  << endl;
	cout << data.angularRate[0] <<  "\t" << data.angularRate[1] << "\t" << data.angularRate[2] << endl;	
	//cout << "Mag " << data.magneticField[0] <<  " " << data.magneticField[1] << " " << data.magneticField[2] << endl;	
	cout <<endl;
}


int spatial::spatial_setup(CPhidgetSpatialHandle &spatial, deque<CPhidgetSpatial_SpatialEventData>* raw, int dataRate)	{
	//Code taken from provided example code "Spatial-simple.c"
	int result;
	const char *err;	

	//Set the handlers to be run when the device is plugged in or opened from software, unplugged or closed from software, or generates an error.
	CPhidget_set_OnAttach_Handler((CPhidgetHandle)spatial, AttachHandler, NULL);
	CPhidget_set_OnDetach_Handler((CPhidgetHandle)spatial, DetachHandler, NULL);
	CPhidget_set_OnError_Handler((CPhidgetHandle)spatial, ErrorHandler, NULL);

	//Registers a callback that will run according to the set data rate that will return the spatial data changes
	//Requires the handle for the Spatial, the callback handler function that will be called, 
	//and an arbitrary pointer that will be supplied to the callback function (may be NULL)
	CPhidgetSpatial_set_OnSpatialData_Handler(spatial, SpatialDataHandler, raw);
	//CPhidgetSpatial_set_OnSpatialData_Handler(spatial, SpatialDataHandler, NULL);

	//open the spatial object for device connections
	CPhidget_open((CPhidgetHandle)spatial, -1);

	//get the program to wait for a spatial device to be attached
	printf("Waiting for spatial to be attached.... \n");

	if((result = CPhidget_waitForAttachment((CPhidgetHandle)spatial, 10000)))
	{
		CPhidget_getErrorDescription(result, &err);
		printf("Problem waiting for attachment: %s\n", err);
		return 0;
	}

	//Display the properties of the attached spatial device
	display_properties((CPhidgetHandle)spatial);

	//Set the data rate for the spatial events
	CPhidgetSpatial_setDataRate(spatial, dataRate);

	cout << "Spatial setup complete" << endl;

	return 0;

}

