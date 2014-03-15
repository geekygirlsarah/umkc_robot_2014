#raw data from the phidget, units converted to standard units
#acceleration in m/s/s (from g's) 
float64 a_x  
float64 a_y 
float64 a_z
#angular rate in rad/s (from deg/s)
float64 w_x
float64 w_y
float64 w_z
#magnetic field in gauss
float64 m_x
float64 m_y
float64 m_z
time timestamp	#timestamp.secs: seconds since attach event
				#timestamp.nsecs: nanosecons since timestamp.secs
