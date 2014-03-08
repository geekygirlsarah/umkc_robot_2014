wesley - 2013/4 UMKC IEEE/ACM Robotics

This branch contains the camera/opencv code that will be used to operate, see,
   and determine the world around the robot.

notes - 01MAR14

id_flame:
	list of locations to check. at least ten.
	these are currently in servo position. need to find the put(xyzp)
	position.
actual  : 27    121     98      33      90      90
actual  : 25    121     98      34      90      90
actual  : 28    121     98      34      90      90
actual  : 25    121     98      35      90      90
actual  : 23    121     97      35      92      90
actual  : 31    121     97      34      92      90
actual  : 26    121     97      34      90      90
actual  : 28    121     97      35      89      90
actual  : 29    121     97      36      89      90
actual  : 21    121     97      36      93      90
actual  : 24    121     97      34      93      90
actual  : 31    121     97      35      88      90


id_tool: FIND_TOOL
-- these positions are from the perspective of the robot.
-- with all of these, like above, more positions can be had.
   stop location of the robot WAS NOT taken into consideration.

	LEFT:
actual  : 51    122     73      5       95      0
actual  : 46    120     73      5       95      0
actual  : 47    120     73      9       95      0
actual  : 58    120     73      9       95      0
actual  : 58    120     73      2       95      0
actual  : 58    120     78      2       95      0
actual  : 48    120     78      2       95      0
actual  : 48    109     84      2       95      0
actual  : 49    105     91      2       95      0
actual  : 56    109     84      2       95      0
actual  : 51    102     84      11      95      0
actual  : 51    128     68      11      95      0
actual  : 55    128     68      11      95      0

	CENTER:
actual  : 83    128     68      11      95      0
actual  : 83    128     68      8       95      0
actual  : 86    123     68      8       95      0
actual  : 80    123     68      8       95      0
actual  : 80    138     59      8       95      0
actual  : 87    138     59      8       95      0
actual  : 80    138     59      7       95      0
actual  : 84    125     59      16      95      0
actual  : 80    125     59      16      95      0
actual  : 80    120     63      16      95      0
actual  : 84    120     63      16      95      0
actual  : 84    116     69      16      95      0
actual  : 80    116     69      11      95      0

	RIGHT:
actual  : 111   116     69      22      95      0
actual  : 109   116     69      22      95      0
actual  : 109   138     57      22      95      0
actual  : 115   123     66      19      95      0
actual  : 110   123     66      19      95      0
actual  : 109   120     70      17      95      0
actual  : 116   120     70      17      95      0
actual  : 112   108     70      27      95      0
actual  : 112   103     74      27      95      0
actual  : 109   103     74      25      95      0
actual  : 109   130     71      7       95      0
actual  : 117   117     78      7       95      0
actual  : 108   117     78      7       95      0



id_tool: FIND_TOP
-- with all of these, like above, more positions can be had.
   stop location of the robot WAS NOT taken into consideration.

	LEFT:
actual  : 51    86      99      5       95      0
actual  : 51    86      110     1       95      0
actual  : 51    95      103     2       95      0
actual  : 53    95      100     2       95      0
actual  : 51    85      103     2       95      0

	CENTER:
actual  : 82    105     86      2       95      0
actual  : 82    97      90      3       95      0
actual  : 79    97      90      3       95      0
actual  : 83    97      90      5       95      0
actual  : 82    97      84      5       95      0
actual  : 81    90      90      5       95      0

	RIGHT:
actual  : 110   90      90      14      95      0
actual  : 112   92      97      14      95      0
actual  : 112   97      97      12      95      0
actual  : 113   97      92      12      95      0
actual  : 111   86      97      13      95      0



after doing some work on paper and the white-board I have determined the
following forward-kinematic equation to produce the position vector R from the
base of the arm to the tip of the gripper.

	the 'CAPITAL_LETTER' words are the defines used in arm_control.h
	the 'lower-case' words are the angles received either from:
		arm[JOINT].read(), or
		position[JOINT] array

	r = < 0, 0, BASE_HGT > 
	  + < HUMERUS*cos(base), HUMERUS*cos(shoulder), HUMERUS*sin(shoulder) >
	  + < ULNA*cos(base),    ULNA*cos(elbow - 90),  ULNA*sin(elbow - 90)  >
	  + < GRIPPER*cos(base), GRIPPER*cos(wrist_p),  GRIPPER*sin(wrist_p)  >

	from this result, we get:

	r = < sumofpieces(x),
	      sumofpieces(y),
	      sumofpieces(z) >

	r = < cos(base) * (HUMERUS, ULNA, GRIPPER), 
		< sqrt( (HUMERUS*cos(shoulder))^2 + (ULNA*cos(elbow - 90))^2 + (GRIPPER*cos(wrist_p))^2 ),
		< sqrt( (HUMERUS*sin(shoulder))^2 + (ULNA*sin(elbow - 90))^2 + (GRIPPER*sin(wrist_p))^2 + BASE_HGT^2 ) >
