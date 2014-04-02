
//THESE ARE IMPORTANT
#define ITERATION 2  //how many gaps to cross.. for debugging
#define LASTGAP 2  //which one to stop at and do the hardcoded one
#define PAUSE_DURATION  300  //how many milliseconds between movements




//THese are just debugging things
//separating the Crossing Wave state
//#define DEBUG_COMMS  //don't test sensor stuf! just the comms!
//#define TEST_TRANSITION_FROM_TOOLS_ONLY   //this starts from the tool pick up position, then goes back to default home position 
#define TEST_CROSS_BOARD_FROM_HOME_ONLY //the opposite of the above will starts from the default home position, goes all the way across - this will supercede the test_transition_from_tools
//#define TEST_TURN_90_ONLY  //ONLY have up if you want to test 90degree stuff D:


/* mega movement tester
 * written by: victoria wu
 * date: 1/17/14 
 *
 * what: make the robot Fkeep going until the gap is found. then stop. (going to put it all on the mega, no bridge just yet
 * FIRST - just getting the robot to move. NO comms between mini and mega. just everything on the mega, ignoring encoders right now
 * PURPOSE: 
 
 * .... pro tip: if you get the error "serial2 is not declared in this scope" check that the board you're programming for IS THE MEGA
 * -> first starting. .the motors just go crazy  a bit 
 
 * next -> turn in place until parallel!! 
 */

//woohoo ros time !
#include <QuadEncoder.h>
#include <ros.h>
#include <mega_caretaker/MegaPacket.h>
#include "redux_mega_packet_defs.h"

#include "navigation.h"

#include <AnalogDistanceSensor.h>
#include <Distance2D120X.h>
#include <DistanceSensor.h>

#include <gapfinder.h>
#include <irsensor_tester.h>
#include <magellan_edgesensors.h>
#include <fronteyes.h>

#include <parallelpark_simple.h>
#include <motor_cmd.h>

#include <movement.h>
#include "top_level_state.h"  //silly arduino doesn't let me use enum as parameter unless it's in another header file T.T

#include "FiniteStateMachine.h"







//=========================
//TOP LEVEL DECLARATIONS
//=========================
//... this is terrible. please fix me
//i need nav declaration to be up here, but FSM declaration needs to go after the state stuff T.T
Navigation nav;
int gapsThru;
FSM stateMachine; //initialize state machine, start in state: waitForCommand
state_top current_status;




//hacky hacky hacky :(
bool handshakeOK;    //used to sync handshake for old state machine
bool turn90DegreeFinished;

bool arrivedAtTools;
bool crossBoard;
bool  commandGoToTools;
bool  isGapFound;
bool  isGapCrossed;
bool  isEdgeFound;
bool travelingHome;





ros::NodeHandle  nh;	
mega_caretaker::MegaPacket temp;
mega_caretaker::MegaPacket advertising_state;  //specifically for advertising state



//pubs and subscribers
ros::Publisher talker("/mega_caretaker/arduinoToBoard", &temp);
void packet_catch(const mega_caretaker::MegaPacket& packet);  
ros::Subscriber<mega_caretaker::MegaPacket> listener("/mega_caretaker/boardToArduino", &packet_catch);

//callback


//=======================
//STATE things
//=======================

void updateROS_spin()  {
  nh.spinOnce();
}

//-----------
//initializeComms - this is for the three way handshake-ing
//-----------

State initializeComms = State(enterInitializeComms, updateInitializeComms, NULL);

void enterInitializeComms()  {
  nav.stopNow();
}

void updateInitializeComms()  {
  updateROS_spin();
  //no talking. just waiting for syn + ack
}


//-----
// waitForCommand = mega is sitting idly, waiting for command from board.
//-----
State waitForCommand = State(enterWaitForCommand, updateWaitForCommand, NULL);  //wait for command from board. either to go somewhere, or start wave crossing.
void enterWaitForCommand()  {

  advertising_state.payload = PL_WAITING;
  talker.publish(&advertising_state);

  nav.stopNow();
  //this is replaced by the board initiated three way handshake
}

void updateWaitForCommand()  {
  
  updateROS_spin();

}


State finishedGoToTools = State(enterFinishedGoToTools, NULL, NULL);
void enterFinishedGoToTools()  {
  sendFinishedGoToTools();
  //stateMachine.transitionTo(waitForCommand);
}


//----
//go to tools! (meta state)
//----

State goToTools = State(enterGoToTools, updateGoToTools, exitGoToTools);
void enterGoToTools()    {
  //this is a meta state??? bleeeeh
  #ifdef DEBUG_COMMS
    digitalWrite(2, HIGH);
  #endif
 
  #ifndef DEBUG_COMMS
  nav.goForwardForever();    //CHAGNDS:FKLJSDKLF:SDJ:FLDKSF
  #endif
  arrivedAtTools = false;
}
void updateGoToTools()  {
  //do the thing! do the thing get the tools the tools
  //say HEY im finished
  //TODO TODO TDOTODTS:DKLFJ:SDKLFJK:SDLf
  
  //
  #ifndef DEBUG_COMMS
  //todo - actually mjoving places
  
  //so right now this is just a straight go forward this number of ticks.
  //TODO put in chase's code to make it actually follow the wall
  
  nav.travelToTools();  //blocks and breaks once # of ticks are there 
  #endif

  arrivedAtTools = true;
 
 
}
void exitGoToTools()  {

}




//----
//meta state - going from position to pick up tools, back to start position (hopefully just goingo backwards. may need to be more robust to
//account for straying too far from the board
//----
State transitionToolsToCrossBoard(enterTransitionToolsToCrossBoard, updateTransitionToolsToCrossBoard, exitTransitionToolsToCrossBoard);
void enterTransitionToolsToCrossBoard()  {
 isEdgeFound = false;
 advertising_state.payload = PL_TRANSITION_1_2;
 talker.publish(&advertising_state);
 nav.goBackwardForever();
}

void updateTransitionToolsToCrossBoard()  {
  //just go backwards until I hit the edge
   //TODO TODO - account for ht elast one nooooooo
   #ifndef DEBUG_COMMS
   isEdgeFound = nav.atEdge();//go "backwards" and find edge
   #endif
   #ifdef DEBUG_COMMS
   isEdgeFound = true;
   #endif
}

void exitTransitionToolsToCrossBoard()  {
}
//-----
// waveCrossing = HEY start crossing waves mate! (meta state)
//-----

State crossingBoard = State(NULL, NULL, NULL);  //wait for command from board. either to go somewhere, or start wave crossing.

//----
//terrible testing class
//-----
State test90DegreeTurn = State(enterTurn90Degrees_cw, updateTurn90Degrees, exitTurn90Degrees);


//------
//turn90Degrees = mega needs to turn 90 degrees now. requires help from board too.
//-------

State turn90Degrees_CW = State(enterTurn90Degrees_cw, updateTurn90Degrees, exitTurn90Degrees);
 void enterTurn90Degrees_cw()  {
 turn90DegreeFinished = false;
 advertising_state.payload = PL_TURNING_CW_INIT;
 talker.publish(&advertising_state);
 nav.stopNow();
 
 //Ask board for help!
                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                       
  initiateTurn90_CW_X_axis();
 }
 
 void updateTurn90Degrees()  {
 updateROS_spin();
 //update other code?? sensor code??
 }
 
 void exitTurn90Degrees()  {
 //?????????
 nav.stop_sleep(PAUSE_DURATION);
 //??????? do i need this???????????????????????????????//
 }
 
 State turn90Degrees_CCW = State(enterTurn90Degrees_ccw, updateTurn90Degrees, exitTurn90Degrees);
 void enterTurn90Degrees_ccw()  {
 turn90DegreeFinished = false;
 advertising_state.payload = PL_TURNING_CCW_INIT;
 talker.publish(&advertising_state);
 nav.stopNow();
 
 //Ask board for help!
 initiateTurn90_CCW_Y_axis();
 }
 
//----------
//GapFound -
//  -needs to turn 90degree CCW
//  -then go forward through the gap, stopping at a certain distance away
//  -then turn 90 degree CW back towards the lane
//----------

State gapFound = State(enterGapFound,NULL,exitGapFound);
 void enterGapFound()  {
 advertising_state.payload = PL_GAP_FOUND;
 talker.publish(&advertising_state);
 
 }
void exitGapFound()  {
  nav.stop_sleep(PAUSE_DURATION);
}


//----------
//lookForGap - travel straight thru lanes, while looking for gap. Stop once we find a gap.
//----------

State lookForGap = State(enterLookForGap, updateLookForGap, exitLookForGap);
 void enterLookForGap()  {
 advertising_state.payload = PL_LOOKING_FOR_GAP;
 talker.publish(&advertising_state);
 nav.takeOff();
 }
 
 //ignoring falling off for now
 void updateLookForGap()  {
 //updateROS_spin();  //do i need this??
 #ifndef DEBUG_COMMS 
 isGapFound = nav.lookingForGap();
  //NEED TO BE MOVING and not stopeed. 
 //stateMachine.immediateTransitionTo(waitForCommand);    
 
 #endif
 #ifdef DEBUG_COMMS
 isGapFound = true;
     
 #endif
 }
 void exitLookForGap()  {
 
 }
 
 //--------------
 //we are finished crossing the gap! in the next lane now.
 //--------------
 State gapCrossed = State(enterGapCrossed, updateGapCrossed, exitGapCrossed);
 void enterGapCrossed()  {
 advertising_state.payload = PL_GAP_CROSSED;
 talker.publish(&advertising_state);
 }
 void updateGapCrossed()  {
 
 }
 void exitGapCrossed()  {
 nav.stop_sleep(PAUSE_DURATION);
 
 }
 //-----------
 //crossGap - make the little crossing across the gap 
 //TODO TODO TODO - account for multiple crossings in a row
 //-------------
 State crossGap = State(enterCrossGap, updateCrossGap, exitCrossGap);
 void enterCrossGap()  {
 advertising_state.payload = PL_CROSSING_GAP;
 talker.publish(&advertising_state);
 gapsThru++;  //gapsThru is the nth gap you have crossed 
 
 }
 void updateCrossGap()  {
 //TODO TODO - account for ht elast one nooooooo
 #ifndef DEBUG_COMMS 
 if(gapsThru == LASTGAP) { //this is the third gap to cross
   //hardcode the ticks
   //block here block block
    nav.crossLastGap();  //blokc block block once its done it will have crossed hopefully
    isGapCrossed = true;
 }
 else  {
   isGapCrossed = nav.crossGap();
 }
 
 //stateMachine.transitionTo(waitForCommand);
 
 #endif
 #ifdef DEBUG_COMMS
 isGapCrossed = true;
 #endif
 
 }
 void exitCrossGap()  {
 nav.stop_sleep(PAUSE_DURATION);
 }
 
//-----------
//Finished wave crossing! yay
//-------------


State finishedCrossingBoard = State(enterFinishedCrossingBoard, updateFinishedCrossingBoard, exitCrossingBoard);
 void enterFinishedCrossingBoard()  {
 sendFinishedCrossingWaves();
 }
 void updateFinishedCrossingBoard()  {
 stateMachine.transitionTo(waitForCommand);
 
 }
 void exitCrossingBoard()  {
 
 }

//-----------
//findEdge- after crossing to another lane and turned clockwise.. go backwards and find edge. 
//-------------

State findEdge = State(enterFindEdge, updateFindEdge, exitFindEdge);
 void enterFindEdge()  {
 advertising_state.payload = PL_FINDING_EDGE;
 talker.publish(&advertising_state);
 nav.goBackwardForever();
 
 }
 
 void updateFindEdge()  {
   //TODO TODO - account for ht elast one nooooooo
   #ifndef DEBUG_COMMS
   isEdgeFound = nav.atEdge();//go "backwards" and find edge
   #endif
   #ifdef DEBUG_COMMS
   isEdgeFound = true;
   #endif
 }
 void exitFindEdge()  {
 
 nav.stop_sleep(PAUSE_DURATION);
 
 }
 




//=======================
//ALL THE ROS THINGS
//=======================      

//ros msg catching time!
void packet_catch(const mega_caretaker::MegaPacket& packet)  {
  //sendAck();  //sd;fjklasd;fljkasd;fkljasdfkl; jd you ack T.T
  
  if(packet.msgType == MSGTYPE_HEY)  {
    if(packet.payload == PL_START_WAVE_CROSSING)  {
   
      //current_status =  start; 
      //gotta put out an ack T.T

      //CHANGE THIS !!!! only for testigngkgadjf;lasdkfjdl;askfj!
      //            start_wave_crossing = true;
      //            stateMachine.immediateTransitionTo(turn90Degrees_CW); 

      // 
      //sendNonsense();
      crossBoard = true;
      // stateMachine.transitionTo(waitForCommand);

    }
    else if (packet.payload == PL_START_GO_TO_TOOLS)  {
      //go to do tools
      //what - when i transition to wait for command here... IT SENDS BACK A SYN ACK WHY
      //stateMachine.transitionTo(waitForCommand);
      commandGoToTools = true;
      
    }
  }
  
    else if(packet.msgType == MSGTYPE_FINISHED)  {
       if(packet.payload == PL_FINISHED_TURNING_90_CW_X_AXIS || packet.payload == PL_FINISHED_TURNING_90_CCW_Y_AXIS) {
           //new thing - immediate transition to finished state  //-> these are only used for crossing  the board.
           turn90DegreeFinished = true; 
         }
        else if(packet.payload == PL_FINISHED_TURNING_90_CW_Y_AXIS || packet.payload == PL_FINISHED_TURNING_90_CCW_X_AXIS)  {  
          //these are used (will be used) in the transition state from rig to crossingboard start)
        }

   }
   
   else if(packet.msgType == MSGTYPE_MOTORCOM)  {
   if(packet.payload == PL_STOP)  {
   //STOP STOP STOP!
   nav.stopNow();  //may need to write in more robust code to keep stopping until...???
   
   }
   else if (packet.payload == PL_TURNCW)  {
   nav.turnClockwiseForever();
   
   }
   else if (packet.payload == PL_TURNCCW)  {
   nav.turnCounterClockwiseForever(); 
   }
   }
   
  else if(packet.msgType == MSGTYPE_HANDSHAKE)    {
    if(packet.payload == PL_SYN)  {

      // handshakeOK = false;
      // stateMachine.immediateTransitionTo(intializeComms);


      temp.msgType = MSGTYPE_HANDSHAKE;
      temp.payload = PL_SYN_ACK;
      talker.publish(&temp);

    }
    else if (packet.payload == PL_ACK)  {
      //connection with board established! 

      //stateMachine.immediateTransitionTo(waitForCommand);

      //current_state = start;
      handshakeOK = true;
    }
  }
}

void sendNonsense()  {
  temp.msgType = 99;
  temp.payload = 9999;
  talker.publish(&temp);
}
/*
void sendAck()  {
  temp.msgType = MSGTYPE_ACK;
  temp.payload = PL_GENERAL_ACK;
  talker.publish(&temp);
}
*/

void sendGoToToolsAck()  {

}



void sendFinishedCrossingWaves()  {
 temp.msgType = MSGTYPE_FINISHED;
 temp.payload = PL_FINISHED_WAVE_CROSSING;
 talker.publish(&temp);
 }
 

void sendFinishedGoToTools()  {
  temp.msgType = MSGTYPE_FINISHED;
  temp.payload = PL_FINISHED_GO_TO_TOOLS;
  talker.publish(&temp);
}

void initROS()  {
  nh.initNode();
  nh.advertise(talker);
  nh.subscribe(listener);

  advertising_state.msgType = MSGTYPE_STATE;


}


void initiateTurn90_CW_X_axis()  {
  temp.msgType = MSGTYPE_HEY;
  temp.payload = PL_START_TURNING_90_CW_X_AXIS;
  talker.publish(&temp);
}


void initiateTurn90_CCW_Y_axis()  {
  temp.msgType = MSGTYPE_HEY;
  temp.payload = PL_START_TURNING_90_CCW_Y_AXIS;
  talker.publish(&temp);
}



//=======================
//MAIn stuff
//=======================




void setup() {
  Serial.begin(9600);
  nav.init();
  current_status = initComms;  //no synack, no rosssSS!
  gapsThru = 0;
  //ros_control = true;
  initROS();
  stateMachine.init(initializeComms);

  handshakeOK = false;
  turn90DegreeFinished = false;
  
  arrivedAtTools = false;
  crossBoard = false; 
  commandGoToTools = false;
  isGapFound = false;
  isGapCrossed = false;
  isEdgeFound = false;
  travelingHome = false;
  
  #ifdef DEBUG_COMMS
    pinMode(2, OUTPUT);
    digitalWrite(2, LOW);
  #endif

}

void loop() {
  //nav.takeOff();
  //start out in waitForcommand.
  //state changes for now (at least) to go to turn90 are handled up there.

  //send the HEY I"m ready to be listening to stuff!!

  // nh.spinOnce();

  //state machine starts out in initializeComms. 
  //-> successfull three way handshake, it goes to 

  //have state machine transitions OUTSIDE here for my sanity


  nh.spinOnce();
  stateMachine.update();
  if(stateMachine.isInState(initializeComms))  {
    //wait and spin
    if(handshakeOK)  {
      stateMachine.transitionTo(waitForCommand);
    }
  }
  else if(stateMachine.isInState(waitForCommand))  {
    //spin
    //will set it to...crossingBoard state
     if(crossBoard)  {
       #ifdef TEST_TURN_90_ONLY
       stateMachine.transitionTo(turn90Degrees_CW);  //TODO change back. this state change is just for testing 90 degree turns without modifying board side ros stuff.
       #else
       stateMachine.transitionTo(crossingBoard);
       #endif
       crossBoard = false;
     } 
     else if (commandGoToTools)  {
       stateMachine.transitionTo(goToTools);
       commandGoToTools = false;
     }
  }
  else if (stateMachine.isInState(goToTools))  {
    if(arrivedAtTools)  {
       stateMachine.transitionTo(finishedGoToTools);
    }
  }
  else if(stateMachine.isInState(finishedGoToTools))  {
    
    stateMachine.transitionTo(waitForCommand);
      
  }
 
    //crossing board state ASSUMES we start from the beginning position     
   else if(stateMachine.isInState(crossingBoard))  {
   //spin
     gapsThru = 0;
      // stateMachine.transitionTo(waitForCommand);
     //stateMachine.transitionTo(finishedCrossingBoard);
     
     //THE VERY FIRST TIME - need to move all the way back to reset.. from looking for tools to going forward. 
     //future optimization - look for Gap 
     
     //i'm at the tools... and need to get back to home
     
     #ifndef TEST_CROSS_BOARD_FROM_HOME_ONLY
     if(!travelingHome)  {
       stateMachine.transitionTo(transitionToolsToCrossBoard);
     }
     #else
     stateMachine.transitionTo(lookForGap);
     #endif
     
     //TODOTODODO - not traveling home change transition./?/    //
     
     
  
   
   //  stateMachine.transitionTo(findEdge);
   
   
   //stateMachine.transitionTo(gapFound);
   }
   
    else if (stateMachine.isInState(transitionToolsToCrossBoard))  {
      
     
      
     if(isEdgeFound)  {
         #ifndef TEST_TRANSITION_FROM_TOOLS_ONLY
         stateMachine.transitionTo(lookForGap);
         #else
         stateMachine.transitionTo(finishedCrossingBoard);
         #endif
     
     }
  }
   
   
   else if(stateMachine.isInState(lookForGap))  {
   //spin.. will transition to gapFound state when its' found
     if(isGapFound)  {
       isGapFound = false;
       //might need to hardcode the ticks if I'm not at an edge.
       #ifndef DEBUG_COMMS
       //NO ADJUSTING
       nav.adjustToGap();
       nav.stop_sleep(PAUSE_DURATION);
       #endif
       //stateMachine.transitionTo(waitForCommand);
   
       stateMachine.transitionTo(gapFound);
     }
   }
   
   else if(stateMachine.isInState(gapFound))  {
   //now need to turn90degrees
   
   #ifndef DEBUG_COMMS

   /*
   //TODO - need to account for waves being at the edge
     if(!nav.atEdge())  {
       nav.adjustToGap();
     }
     */
   #endif
     stateMachine.transitionTo(turn90Degrees_CCW);
     
   
   }
   
   else if (stateMachine.isInState(turn90Degrees_CCW)) {
   //spin... 
   if(turn90DegreeFinished)  {
   //stateMachine.transitionTo(waitForCommand);    //want to se if this works :(
   
   //gapsThru++;    //in position to cross yet another gap - if this is the 3rd one.. DON"T CROSS GAP just go forward and stop
   //if(gapsThru == 1)  {
   //  stateMachine.transitionTo(finishedCrossingBoard);
   //}
   //else  {
   // stateMachine.transitionTo(crossGap);   
   //}
   
     #ifdef TEST_TURN_90_ONLY
      stateMachine.transitionTo(waitForCommand);
     #else
     stateMachine.transitionTo(crossGap);
     #endif
      turn90DegreeFinished = false;
     }
   }
   
   
   //stop here !!
   else if (stateMachine.isInState(crossGap))  {
   //spin
   //the update function in this state will transition once the gap is crossed and it ses a wave in front //TODO TODO update with sarahs code
     if(isGapCrossed)  {
       stateMachine.transitionTo(gapCrossed);
     }
   }
   else if (stateMachine.isInState(gapCrossed) )  {
       
       stateMachine.transitionTo(turn90Degrees_CW);
   
     
   //stateMachine.transitionTo(waitForCommand);
   
   }
   else if (stateMachine.isInState(turn90Degrees_CW))  {
   //spin
     if(turn90DegreeFinished)  {
   //and back to looking for gap
   //stateMachine.transitionTo(waitForCommand);
   //stateMachine.transitionTo(lookForGap);
   
   //-> go backwards until you hit an edge... then look for gap
       #ifdef TEST_TURN_90_ONLY
      stateMachine.transitionTo(waitForCommand);
       #else
       stateMachine.transitionTo(findEdge);
       #endif
       turn90DegreeFinished = false;
   }
   }
   else if (stateMachine.isInState(findEdge))  {
   //???
   //... THIS IS SO hACKY D: -> TODO make it so magellans are active all the time?????? ndl;fkasjdfl;jkasd;fkljasfl;kasjdfkl;ajdfkl
   //spin
     //stateMachine.transitionTo(finishedCrossingBoard);
     if(isEdgeFound)  {
       if(gapsThru < ITERATION)  {
         stateMachine.transitionTo(lookForGap);
       }
       else  {
         stateMachine.transitionTo(finishedCrossingBoard);
       }
     }
   }



  /*
  Serial.print("FL: ");
   Serial.println(positionFL);
   Serial.print(" BL: ");
   Serial.println(positionBL);
   Serial.print(" FR: ");
   Serial.println(positionFR);
   Serial.print(" BR: ");
   Serial.println(positionBR);  
   */

}




