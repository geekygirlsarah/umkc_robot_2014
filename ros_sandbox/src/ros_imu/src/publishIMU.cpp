//TODO
//!!!! Bit of drift... 1-3 degrees in a minute, varies from yaw to pitch.
//Handling spatial timeout
//wait for service?
//-> detaching phidget, then attaching? blocking?

#include "ros/ros.h"
#include "std_msgs/String.h"
#include <ros_imu/spatialRaw.h>
#include "phidget_headers/spatial_helper.h"
#include <ros_imu/orientation.h>
#include <ros_imu/imu_filter.h>
#include <tf/transform_broadcaster.h>
#include "config.h"
#include <ros/console.h>
extern pthread_mutex_t mutex; 	//used when handling data q
extern pthread_cond_t cond;		//used when handling data  q
using namespace std;

bool tacit = false;		// for --quiet flag. defaults to false to display calculated values
double GYRO_BIAS[3];	


#define VERBOSE if (!tacit)
/*
 * Account for bias in gyro 
 */
void adjustBias(ros_imu::spatialRaw *msg)	{
/*
	msg->w_x = msg->w_x - GYRO_OFFSET_R[0]; 
	msg->w_y = msg->w_y - GYRO_OFFSET_R[1]; 
	msg->w_z = msg->w_z - GYRO_OFFSET_R[2]; 
*/
	msg->w_x = msg->w_x - GYRO_BIAS[0]; 
	msg->w_y = msg->w_y - GYRO_BIAS[1]; 
	msg->w_z = msg->w_z - GYRO_BIAS[2]; 
}


/*
 * copySpatialRaw
 * Copies the given spatialraw message into the queue at specified position.
 */
void copySpatialRaw(spatial::PhidgetRawDataQ::iterator it, ros_imu::spatialRaw *msg)	{
	//Subtract bias from gyro.
	//I'm REALLY not sure why there's two of these calls. O.o
	//adjustBias(msg);

	//Copying Timestamp
	msg->timestamp.sec = it->timestamp.seconds;
	msg->timestamp.nsec = it->timestamp.microseconds;

	//Convert Acceleration from g's -> m/s/s, then copy
	msg->a_x = mss_from_gs(it->acceleration[0]);
	msg->a_y = mss_from_gs(it->acceleration[1]);
	msg->a_z = mss_from_gs(it->acceleration[2]);

	//Convert Angular rate from deg/s -> rad/s, then copy
	msg->w_x = rad_from_deg(it->angularRate[0]);
	msg->w_y = rad_from_deg(it->angularRate[1]);
	msg->w_z = rad_from_deg(it->angularRate[2]);

	//Copying Magnetic field (in gauss)
	msg->m_x = it->magneticField[0];
	msg->m_y = it->magneticField[1];
	msg->m_z = it->magneticField[2];

	adjustBias(msg);

	VERBOSE ROS_INFO("ROS Side, Raw Phidget Data");
	VERBOSE ROS_INFO("a_x: %f, a_y: %f, a_z: %f", it->acceleration[0],it->acceleration[1], it->acceleration[2]);
	VERBOSE ROS_INFO("w_x: %f, w_y: %f, w_z: %f", it->angularRate[0],it->angularRate[1], it->angularRate[2]);

	VERBOSE ROS_INFO("Copied Spatial Data");
	VERBOSE ROS_INFO("a_x: %f, a_y: %f, a_z: %f", msg->a_x, msg->a_y, msg->a_z);
	VERBOSE ROS_INFO("w_x: %f, w_y: %f, w_z: %f", msg->w_x, msg->w_y, msg->w_z);


}

/*
 * Gets a spatialraw message from the queue.
 */
void fillSpatialMsg(spatial::PhidgetRawDataQ::iterator it, spatial::PhidgetRawDataQ* dataQueue, ros_imu::spatialRaw *msg)	{
	VERBOSE ROS_INFO("Filling up msg");
	//Dealing with data Queue
	pthread_mutex_lock(&mutex);

	//Check to see if dataQueue is empty. 
	//If it is, wait.
	if(dataQueue->empty())	{
		pthread_cond_wait(&cond, &mutex);
	}
	
	it = dataQueue->begin();
	copySpatialRaw(it, msg);
	dataQueue->pop_front();
	pthread_mutex_unlock(&mutex);


}

int main(int argc, char* argv[]){
	if (argc == 4) {	// change this to 2 if running from rosrun, 4 if from launch file
		if (strcmp(argv[1], "--quiet") == 0) {
			tacit = true;
			ROS_INFO("caterpillar mode engaged; we are go for silent running");
		}
	}

	//ROS Setup
	//-------------------------------
	ROS_INFO("Initializing IMU talker");
  	int ROSbufferSize = 10, ROScount = 0;
 	ros::init(argc, argv, "Phidget_Stuff");
  	ros::NodeHandle PhidgetNode;
  	ros::Rate loop_rate(10);

  	//Getting bias params
//	double gyro_x_bias, gyro_y_bias, gyro_z_bias;
	PhidgetNode.param<double>("gyro_x_bias", GYRO_BIAS[0], -0.00494777);
	PhidgetNode.param<double>("gyro_y_bias", GYRO_BIAS[1], -0.00370861);
	PhidgetNode.param<double>("gyro_z_bias", GYRO_BIAS[2], -0.0013638);

	ROS_INFO("Gyro X Bias - checking %f", GYRO_BIAS[0]);

	//Setting up publishers
	ros::Publisher PhidgetPub = 
    	PhidgetNode.advertise<ros_imu::spatialRaw>("IMU_data", ROSbufferSize);
	ros::Publisher rpyPub = PhidgetNode.advertise<ros_imu::orientation>("RPY_data", ROSbufferSize);
	ros::Publisher RotMatrixPub = PhidgetNode.advertise<ros_imu::rotMatrix>("Rotation_Matrix", ROSbufferSize);
	ros::Publisher orientationPub = PhidgetNode.advertise<geometry_msgs::Quaternion>("Orientation_data", ROSbufferSize);
	
	ROS_INFO("Publishers Set up");

	//Connecting to Service
	ros::ServiceClient client = PhidgetNode.serviceClient<ros_imu::imu_filter>("Calculate_Orientation");

	//Setting up transform broadcaster
	tf::TransformBroadcaster broadcaster;
	
	//Creating/Initializing Spatial Handle
	//------------------------------------
	ROS_INFO("Initializing Spatial handles");
	CPhidgetSpatialHandle spatial =0;
	CPhidgetSpatial_create(&spatial);

	//Setting up data q
	//------------------------------------
	spatial::PhidgetRawDataQ* dataQueue = new spatial::PhidgetRawDataQ();
	spatial::PhidgetRawDataQ::iterator it = dataQueue->begin();
	
	//init mutex
	//-------------------------------------
	if(pthread_mutex_init(&mutex, NULL)!= 0)	{
		ROS_ERROR("mutex init failed");
		return -1;	//erm this is bad
	}
	else	{
		ROS_INFO("mutex init success \n");
	}

	//init cond
	//-----------------------------------
	if(pthread_cond_init(&cond, NULL)!=0)	{
		ROS_ERROR("cond init failed!");
	}
	else	{
		ROS_INFO("cond init success \n");
	}
	
	//set up spatial
	//-------------------------------------
	ROS_INFO("Setting up Spatial");
	spatial::spatial_setup(spatial, dataQueue, DATA_RATE);

 	while(ros::ok()) {
		

		//Publishing raw IMU data
		ros_imu::spatialRaw  msg;
			
		fillSpatialMsg(it, dataQueue, &msg);

	//	ROS_INFO("Time %ds %dns", msg.timestamp.sec, msg.timestamp.nsec);
	//	ROS_INFO("Gyr X:%f Y:%f Z:%f", msg.w_x, msg.w_y, msg.w_z);	
		PhidgetPub.publish(msg);
	
		//Publishing orientation data
		
		ros_imu::imu_filter srv;
		srv.request.rawIMU = msg;
		//Update filter with newest data.
		if(client.call(srv))	{
			VERBOSE ROS_INFO("Orientation_Calculate call successful.");
			VERBOSE ROS_INFO("Roll: %f", srv.response.rpy.roll);
			VERBOSE ROS_INFO("pitch: %f", srv.response.rpy.pitch);
			VERBOSE ROS_INFO("yaw: %f", srv.response.rpy.yaw);
			
			//Publishing data
			rpyPub.publish(srv.response.rpy);
			RotMatrixPub.publish(srv.response.rot);
			orientationPub.publish(srv.response.orientation);	
			
			//Broadcasting TF transform
			//from ccny-ros-pkg/imu_tools repository
	/*
			tf::Quaternion q(srv.response.orientation.x,
							srv.response.orientation.y,
							srv.response.orientation.z,
							srv.response.orientation.w);
			tf::Transform transform;
			transform.setOrigin( tf::Vector3(0,0,0));
			transform.setRotation(q);
			broadcaster.sendTransform(
				tf::StampedTransform(
					transform, 
					srv.response.pose.header.stamp,
					"world",
					 "IMU")
				);
				*/
		}
		else	{
			ROS_ERROR("Failed to call service Calculate_orientation");
		}
	
    	ROScount++;
//		loop_rate.sleep();
    	ros::spinOnce();
 	}
}
