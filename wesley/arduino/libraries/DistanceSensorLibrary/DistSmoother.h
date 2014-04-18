/*
 * UMKC 2014 Robotics
 *
 * Uses the Distance2D120X sensor library.
 * Is a wrapper class to provide smoothing and filtering.
 *
 * Focuses on filtering out abrupt spikes in data
 
 getFilteredDistanceCM is not accurate, don't use it. use getAccurateDistCM() instead
 
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

//taken from ->, just substututing in our own function to getnice distances
/// Arduino library for distance sensors
// Copyright 2011-2013 Jeroen Doggen (jeroendoggen@gmail.com)
/// isCloser: check whether the distance to the detected object is smaller than a given threshold
boolean isCloser(int threshold)
{
  if (threshold>getAccurateDistCM())
  {
    return (true);
  }
  else
  {
    return (false);
  }
}

/// isFarther: check whether the distance to the detected object is bigger than a given threshold
boolean isFarther(int threshold)
{
  if (threshold<getAccurateDistCM())
  {
    return true;
  }
  else
  {
    return false;
  }
}


int getAccurateDistCM()
				{
					int size = 10;
					int size2 = 10;
					float SD;
					float mean;
					float SDsum = 0;
					float meanRef = 0;
					float readingArr[size];
					float finalArr[size2];
					int j = 0;
					for(int i = 0; i < size; i++)
					{
						readingArr[i] = dist.getDistanceCentimeter();
					}
					for(int i = 0; i < size; i++)
					{
						meanRef += readingArr[i];
					}
					mean = meanRef/size;
					for(int i = 0; i < size; i++)
					{
						SDsum += (readingArr[i] - mean)*(readingArr[i] - mean);
					}
					SD = sqrt(0.1*SDsum);
					for(int i = 0; i < size; i++)
					{
						if(readingArr[i] > (mean+2*SD) || readingArr[i] < (mean-2*SD))
						{
							size2--;
						}
						else
						{
							finalArr[j] = readingArr[i];
							j++;
						} 
					}
					meanRef = 0;
					for(int i = 0; i < size2; i++)
					{
						meanRef += finalArr[i];
					}
					mean = meanRef/size2;
					return mean;
				}


};

#endif
