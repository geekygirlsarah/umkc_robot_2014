umkc_robot_2014_arduino
=======================

Arduino code for the UMKC IEEE Robotics Team 2014 IEEE R5 Competition

DEVELOPMENT NOTES
=================

The manner in which you develop is your own; by all means, be as productive as you can.

I would like to highlight some co-development rules:
  - do not make severe changes to someone else's code base without first consulting the
    original author / developer.
  - adding an overload to add or remove functionality is fine. removing, or rewriting,
    entire sections or a class is not.
  - If you have any questions regarding the use of a library / class, ask the developer.
  - do not move files or alter the behavior of another's class.

Again, feel free to develop in whatever manner you see fit; the choice of IDE, location
    of files, layout / structure of classes, and definition of variables can be a very
    personal decision. But by the time your code hits the MASTER branch, it should conform
    to the following:
    
    - DeclarationOfClass,     camel-case w/leading Capitalization
    - declarationOfMethods,   camel-case w/o leading capital
    - /* pre- and post-conditional commentary
    - an explanation regarding the use of any constants
    - example code showing use of public functions
    - at the head of your source files:
    
      /* NAMEOFFILE.(CPP|H|etc..)
       * written by: All persons who had a hand in writing this file
       * date: generally of original writing, but can be adjusted as needed
       *
       * PURPOSE: A sentence or two explaining the purpose of the file.
       *
       * TODOs, or any other notes that you feel are necessary to describe the source
       *
       */
       
    - If your code references a self-written header file, it must be included in the same
      directory as the source-code. No consolidation of header files.

branch status
=======
* master - tested working stuff.
* move - how to combine ir sensor data to make robo move intelligently
* arm - basic framework for arm movement includiung an inverse kinematic function


resources
======
library for ir sensors we are using - https://code.google.com/p/gp2y0a21yk-library/ (NOPE) 
updated to -> https://github.com/jeroendoggen/arduino-distance-sensor-library

documentation for that library - https://gp2y0a21yk-library.googlecode.com/hg/Documentation/latex/refman.pdf

sharp ir sensor diagram - http://www.sharpsma.com/webfm_send/1203

Encoder library - http://pjrc.com/teensy/td_libs_Encoder.html
