#ifndef PHIDGET_STUFF_H
#define PHIDGET_STUFF_H

#include <cstdlib>
#include <pthread.h>

//Mutex, used with event handler in setting up the shared std::deque
pthread_mutex_t mutex;
pthread_cond_t cond;
//callback that will run if the Spatial is attached to the computer
int CCONV AttachHandler(CPhidgetHandle spatial, void *userptr)
{
	int serialNo;
	CPhidget_getSerialNumber(spatial, &serialNo);
	printf("Spatial %10d attached!", serialNo);

	return 0;
}

//callback that will run if the Spatial is detached from the computer
int CCONV DetachHandler(CPhidgetHandle spatial, void *userptr)
{
	int serialNo;
	CPhidget_getSerialNumber(spatial, &serialNo);
	printf("Spatial %10d detached! \n", serialNo);

	return 0;
}

//callback that will run if the Spatial generates an error
int CCONV ErrorHandler(CPhidgetHandle spatial, void *userptr, int ErrorCode, const char *unknown)
{
	printf("Error handled. %d - %s \n", ErrorCode, unknown);
	return 0;
}

//Display the properties of the attached phidget to the screen.  
//We will be displaying the name, serial number, version of the attached device, the number of accelerometer, gyro, and compass Axes, and the current data rate
// of the attached Spatial.
int display_properties(CPhidgetHandle phid)
{
	int serialNo, version;
	const char* ptr;
	int numAccelAxes, numGyroAxes, numCompassAxes, dataRateMax, dataRateMin;

	CPhidget_getDeviceType(phid, &ptr);
	CPhidget_getSerialNumber(phid, &serialNo);
	CPhidget_getDeviceVersion(phid, &version);
	CPhidgetSpatial_getAccelerationAxisCount((CPhidgetSpatialHandle)phid, &numAccelAxes);
	CPhidgetSpatial_getGyroAxisCount((CPhidgetSpatialHandle)phid, &numGyroAxes);
	CPhidgetSpatial_getCompassAxisCount((CPhidgetSpatialHandle)phid, &numCompassAxes);
	CPhidgetSpatial_getDataRateMax((CPhidgetSpatialHandle)phid, &dataRateMax);
	CPhidgetSpatial_getDataRateMin((CPhidgetSpatialHandle)phid, &dataRateMin);

	

	printf("%s\n", ptr);
	printf("Serial Number: %10d\nVersion: %8d\n", serialNo, version);
	printf("Number of Accel Axes: %i\n", numAccelAxes);
	printf("Number of Gyro Axes: %i\n", numGyroAxes);
	printf("Number of Compass Axes: %i\n", numCompassAxes);
	printf("datarate> Max: %d  Min: %d\n", dataRateMax, dataRateMin);
	return 0;
}

//callback that will run at datarate
//data - array of spatial event data structures that holds the spatial data packets that were sent in this event
//count - the number of spatial data event packets included in this event
int CCONV SpatialDataHandler(CPhidgetSpatialHandle spatial, void *userptr, CPhidgetSpatial_SpatialEventDataHandle *data, int count)
{
	//Making copy of data.
	//--------------------

	CPhidgetSpatial_SpatialEventData* dataHolder = (CPhidgetSpatial_SpatialEventData*)malloc(sizeof(CPhidgetSpatial_SpatialEventData));

	dataHolder = spatial::copy(*data[0]);
	// emg - muted
//	spatial::print(*dataHolder);

	//Pushing data to user given vector.
	//---------------------
	std::deque<CPhidgetSpatial_SpatialEventData>* buffer = (std::deque<CPhidgetSpatial_SpatialEventData>*) userptr;
	

	
	//Get rid of oldest data, push in new one.
	pthread_mutex_lock(&mutex);
	buffer->push_back(*dataHolder);
//	printf("Pushing into dataQueue\n");
//	spatial::print(buffer->back());
	//buffer->pop_front();
	
	//does this need to be in an if???
	pthread_cond_signal(&cond);
	pthread_mutex_unlock(&mutex);

	//Freeing up memory of dataholder
	free(dataHolder);

	return 0;
}


/*PHIDGET_STUFF_H*/
#endif
