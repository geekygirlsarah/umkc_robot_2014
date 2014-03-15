#ifndef CONFIG_H
#define CONFIG_H


#define gyroscopeErrorRate  0.01

#define mss_from_gs(X) ((X)*9.8)
#define rad_from_deg(X) ((X)*.0174532925)
#define deg_from_rad(X) ((X)* 57.2957795)
#define seconds_from_ms(x) ((x)/1000.0)
#define DATA_RATE 16	//phidget pushes packets every 16 millisec
//BURRIS
const double GYRO_OFFSET_B[3] = {-0.00494777, -0.00370861, -0.0013638};

//ROBOTICS - tested on table, 10 min, in deg/s
const double GYRO_OFFSET_R[3] = {-0.004615, -0.000150139, -0.00632318};

/*SPATIAL_CONFIG_H*/
#endif
