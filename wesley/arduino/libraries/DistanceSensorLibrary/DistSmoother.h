/*
 * UMKC 2014 Robotics
 *
 * Uses the Distance2D120X sensor library.
 * Is a wrapper class to provide smoothing and filtering.
 *
 * Focuses on filtering out abrupt spikes in data
 *
 *
 * brain dump
 *
 * mark being the number of times i've spotted an outlier
*
 * repeat outerloop # of times
 * 	take reading
 * 	if not outside bound
 * 		alpha it in
 *  else
 *  	mark ++ and alpha it separately
 *
 *  if marknum > tolerated
 *  	alpha those. that's the new baseline
 *
 */

#ifndef DISTSMOOTHER_H
#define DISTSMOOTHER_H

#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#include <pins_arduino.h>
#endif

#include "Distance2D120X.h"


class DistSmoother	{

		private:
				Distance2D120X dist;
				
				float alpha;	//how much do you value current data over previous data
				int numReadings;	//how many readings to do?
				int numToleratedOutliers;	//how many outliers must be seen in those readings to make them NOT outliers?
				int numCurrentOutliers; //how many outliers i've seen so far in this reading chunk	

				int currentReading; //raw reading from sensor

				int smoothedReading;
				int smoothedReadingOutlier;

				int baseline;		
				int halfWidth;	//how wide the gap is for not noisy data [baseline -halfwidth,  baseline + halfwidth]

				const int static numReadings_default = 5;
				const int static numToleratedOutliers_default = 3;
				const float static alpha_default = .8;
				const int static halfWidth_default= 3;

				bool withinBaseline(int current)	{
					return (current > (baseline - halfWidth) && current < (baseline + halfWidth));	
				}

		public:
				DistSmoother()	{

				};

				void init(int pin)	{
					dist.begin(pin);

					numReadings = numReadings_default;
					numToleratedOutliers = numToleratedOutliers_default;
					alpha = alpha_default;
					halfWidth = halfWidth_default;

					smoothedReading = 0;
					smoothedReadingOutlier = 0;
				}

				int getRawCM()	{
					return dist.getDistanceCentimeter();
				}

				int getFilteredDistanceCM()	{

					currentReading = dist.getDistanceCentimeter(); 	
					smoothedReading = currentReading;
					smoothedReadingOutlier = currentReading;
					//read it in numReadings  number of times.
					for(int i =0; i< numReadings; i++)	{
						currentReading = dist.getDistanceCentimeter();
						if(withinBaseline(currentReading))	{
							smoothedReading = (alpha*currentReading) + (1-alpha)*(smoothedReading);	
						}
						else	{
							//increase the mark
							numCurrentOutliers++;
							smoothedReadingOutlier = (alpha*currentReading) + (1-alpha)*(smoothedReadingOutlier);	
						}
					}

					if(numCurrentOutliers > numToleratedOutliers)	{
						//change baseline to new outlier
						baseline = smoothedReadingOutlier;
					}
					else	{
						baseline = smoothedReading;
					}
					return baseline;
				}
};

#endif
