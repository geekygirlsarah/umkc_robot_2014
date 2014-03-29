//WALL FOLLOWER V2//


// CHASE PETERSON //


#include "DistSmoother.h"
#include <motor_cmd.h>
#include <stdlib.h>


#ifndef WALLFOLLOWER2_H
#define WALLFOLLOWER2_H


class WallFollow
{

private:
	motor_cmd* S;
	enum internal  {good, list_right, list_left};
	
	internal straightness;
	
	DistSmoother front, middle, back;
	bool notGood;
	int UPPER_LIMIT, LOWER_LIMIT, MIDDLE;
	
public:

	WallFollow() { };

	void setLimits(int far, int mid, int close)
	{
		UPPER_LIMIT = far;
		MIDDLE = mid;
		LOWER_LIMIT = close;
	}
	
	void init(int pin1 , int pin2, int pin3, motor_cmd*  s) 
	{
		front.init(pin1);
		middle.init(pin2);
		back.init(pin3);
		S = s;
		straightness = good;
		
		notGood = false;
	}
	
	internal straight()
		{
		internal result;
		notGood = false;
		if(front.getAccurateDistCM() > 25)
			{
				if(middle.getAccurateDistCM() > UPPER_LIMIT)
				{
					Serial.print("Middle sensor reads: ");
					Serial.println(middle.getAccurateDistCM());
					notGood = true;
					Serial.println("TOO FAR AWAY");
					return list_right;
				}
				else if(middle.getAccurateDistCM() < LOWER_LIMIT)
				{
					Serial.print("Middle sensor reads: ");
					Serial.println(middle.getAccurateDistCM());
					notGood = true;
					Serial.println("TOO CLOSE");
					return list_left;
				}
				
			}
			
		else if(middle.getAccurateDistCM() > 25)
			{
				if(front.getAccurateDistCM() > UPPER_LIMIT)
				{
					Serial.print("Front sensor reads: ");
					Serial.println(front.getAccurateDistCM());
					notGood = true;
					Serial.println("TOO FAR AWAY");
					return list_right;
				}
     
				else if(front.getAccurateDistCM() < LOWER_LIMIT)
				{
					Serial.print("Front sensor reads: ");
					Serial.println(front.getAccurateDistCM());
					notGood = true;
					Serial.println("TOO CLOSE");
					return list_left;
				}
			}
		else if(back.getAccurateDistCM() > 25)
			{
				if(front.getAccurateDistCM() > UPPER_LIMIT)
				{
					Serial.print("Front sensor reads: ");
					Serial.println(front.getAccurateDistCM());
					notGood = true;
					Serial.println("TOO FAR AWAY");
					return list_right;
				}
				else if(front.getAccurateDistCM() < LOWER_LIMIT)
				{
					Serial.print("Front sensor reads: ");
					Serial.println(front.getAccurateDistCM());
					notGood = true;
					Serial.println("TOO CLOSE");
					return list_left;
				}
			}	
  
	
		else
			{
				if(front.getAccurateDistCM() > UPPER_LIMIT)
				{
					Serial.print("Front sensor reads: ");
					Serial.println(front.getAccurateDistCM());
					notGood = true;
					Serial.println("TOO FAR AWAY");
					return list_right;
				}
				else if(front.getAccurateDistCM() < LOWER_LIMIT)
				{
					Serial.print("Front sensor reads: ");
					Serial.println(front.getAccurateDistCM());
					notGood = true;
					Serial.println("TOO CLOSE");
					return list_left;
				}
			}
		return result;
		}
		
	
	internal update(const byte DIRECTION)
	{
		int outerloopcount = 1;
		while(outerloopcount-- > 0)
		{
			switch(straightness)
			{
				case good:
					if(straight() == list_left)
						{
						straightness = list_left;
						if(DIRECTION == S->REVERSE)
							S->reverse_correct_right();
						else if(DIRECTION == S->FORWARD)
							S->forward_correct_right();
						}
					else if(straight() == list_right)
						{
						straightness = list_right;
						if(DIRECTION == S->REVERSE)
							S->reverse_correct_left();
						else if(DIRECTION == S->FORWARD)
							S->forward_correct_left();
						}
					else
						{
						if(DIRECTION == S->REVERSE)
							S->reverse();
						else if(DIRECTION == S->FORWARD)
							S->forward();
						}
					break;
				case list_left:
					if(straight() == good)
						{
						straightness = good;
						if(DIRECTION == S->REVERSE)
							S->reverse();
						else if(DIRECTION == S->FORWARD)
							S->forward();
						}
					else if(straight() == list_right)
						{
						straightness = list_right;
						if(DIRECTION == S->REVERSE)
							S->reverse_correct_left();
						else if(DIRECTION == S->FORWARD)
							S->forward_correct_left();
						}
					else
						{
						straightness = list_left;
						if(DIRECTION == S->REVERSE)
							S->reverse_correct_right();
						else if(DIRECTION == S->FORWARD)
							S->forward_correct_right();
						}
					break;
				case list_right:
					if(straight() == good)
						{
						straightness = good;
						if(DIRECTION == S->REVERSE)
							S->reverse();
						else if(DIRECTION == S->FORWARD)
							S->forward();
						}
					if(straight() == list_left)
						{
						straightness = list_left;
						if(DIRECTION == S->REVERSE)
							S->reverse_correct_right();
						else if(DIRECTION == S->FORWARD)
							S->forward_correct_right();
						}
					else 
						{
						straightness = list_right;
						if(DIRECTION == S->REVERSE)
							S->reverse_correct_left();
						else if(DIRECTION == S->FORWARD)
							S->forward_correct_left();
						}
					break;
				return straightness;
			}
		}
	}
};

#endif
