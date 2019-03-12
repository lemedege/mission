/***************************************************************************
 *   Copyright (C) 2016 by DTU (Christian Andersen)                        *
 *   jca@elektro.dtu.dk                                                    *
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU Lesser General Public License as        *
 *   published by the Free Software Foundation; either version 2 of the    *
 *   License, or (at your option) any later version.                       *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU Lesser General Public License for more details.                   *
 *                                                                         *
 *   You should have received a copy of the GNU Lesser General Public      *
 *   License along with this program; if not, write to the                 *
 *   Free Software Foundation, Inc.,                                       *
 *   59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.             *
 ***************************************************************************/



#include <sys/time.h>
#include <cstdlib>

#include "umission.h"
#include "utime.h"


UMission::UMission(UBridge * regbot, UCamera * camera)
{
  cam = camera;
  bot = regbot;
  threadActive = 100;
  // initialize line list to empty
  for (int i = 0; i < MAX_LINES; i++)
  { // terminate c-strings strings - good practice, but not needed
    lines[i][0] = '\0';
    // add to line list 
    lineList[i] = lines[i];    
  }
  // start mission thread
  th1 = new thread(runObj, this);
  
}


UMission::~UMission()
{
  printf("Mission class finished\n");
}

// typedef enum  {
//   MSG_HART_BEAT,
//   MSG_POSE,
//   MSG_IR,
//   MSG_EDGE,
//   MSG_ACC,
//   MSG_GYRO,
//   MSG_MOTOR_CURRENT,
//   MSG_WHEEL_VEL,
//   MSG_MAX} MESSAGE_TYPES;
  

void UMission::run()
{
  while (not active and not th1stop)
    usleep(100000);
  if (not th1stop)
    runMission();
  printf("mission thread ended\n");
}
  
  
  
void UMission::missionInit()
{ // stop any not-finished mission
  bot->send("robot stop\n");
  // clear old mission
  bot->send("robot <clear\n");
  //
  // add new mission with 3 threads
  // one (100) starting at event 30 and stopping at event 31
  // one (101) starting at event 31 and stopping at event 30
  // one (  1) used for idle and initialisation of hardware
  // the mission is started, but staying in place (velocity=0, so servo action)
  //
  /* idle thread */
  bot->send("robot <add thread=1\n");
  // Irsensor should be activated a good time before use 
//   printf("UMission::missionInit 3\n");
  // otherwise first samples will produce "false" positive (too short/negative).
  bot->send("robot <add irsensor=1,vel=0:dist=0.2\n");
  //
  // alternating threads (100 and 101, alternating on event 30 and 31 (last 2 events)
  bot->send("robot <add thread=100,event=30 : event=31\n");
  for (int i = 0; i < missionLineMax; i++)
    // send placeholder lines, that will never finish
    // are to be replaced with real mission
    // NB - hereafter no lines can be added to these threads, just modified
    bot->send("robot <add vel=0 : time=1\n");
  //
  bot->send("robot <add thread=101,event=31 : event=30\n");
  for (int i = 0; i < missionLineMax; i++)
    // send placeholder lines, that will never finish
    bot->send("robot <add vel=0\n");
  usleep(10000);
  //
  printf("UMission::missionInit: base mission loaded\n");
  //
  // send subscribe to bridge to the same data (priority [1..4] 1 is fastest, 6 is when new data is available)
  bot->pose->subscribe();
  bot->edge->subscribe();
  bot->motor->subscribe();
  bot->event->subscribe();
  bot->joy->subscribe();
  bot->motor->subscribe();
  bot->info->subscribe();
  // wait a bit for message streams to start
}


void UMission::missionSendAndRun(const char ** missionLines, int missionLineCnt)
{
  // Calling missionSendAndRun automatically toggles between thread 100 and 101. 
  // Modifies the currently inactive thread and then makes it active. 
  const int MSL = 100;
  char s[MSL];
  int threadToMod = 101;
  int startEvent = 31;
  bot->event->clearEvents();
  if (threadActive == 101)
  {
    threadToMod = 100;
    startEvent = 30;
  }
  for (int i = 0; i < missionLineCnt; i++)
  { // send lines one at a time
    if (strlen((char*)missionLines[i]) > 0)
    { // send a modify line command
      snprintf(s, MSL, "<mod %d %d %s\n", threadToMod, i+1, missionLines[i]);
      bot->send(s); 
    }
    else
      break;
  }
  usleep(20000);
  // debug
  // printf("Sending event %d\n",startEvent);
  // debug end
  // Activate this thread and stop the other  
  snprintf(s, MSL, "<event=%d\n", startEvent);
  // wait a bit (20ms) befor sending start event
  bot->send(s);
  threadActive = threadToMod;
}


//////////////////////////////////////////////////////////

void UMission::runMission()
{
  int mission = fromMission;
  int state = 0;
  bool started = false;
  bool wait4running = true;
  bool running = false;
  bool ended = false;
  int loop = 0;
  UTime t;
  int missionState = 0, missionStateOld = 0;;
  // initialize robot mission to do nothing (wait for mission lines)
  missionInit();
  sleep(1);
  t.now();
  printf("%ld.%06ld before start\n", t.getSec(), t.getMilisec());
  bot->send("start\n"); // ask robot to start controlled run (ready to execute)
  while (not finished and not th1stop)
  {
    loop++;
    // handle joystick control
//     inManualControl = joy->testJoy(inManualControl);
    // manuel RC control is in state 0
    if (bot->joy->button[1] and not cam->saveImage)
    { // red button 
      printf("UMission::runMission:: button 1 (red) save image\n");
      cam->saveImage = true;
    }
    if (bot->joy->manual)
    { // just wait, do not continue mission
      usleep(20000);
    }
    else
    { // in auto mode, so start or continue
      switch(state)
      {
        case 0: // waiting for start button or manual control
          // (button 0=green, 1=red, blue=2, yellow=3, start=7, back=6, LB=4, RB=5
//           if (bot->joy.button[1])
//           { // red button 
//             printf("UMission::runMission:: button 1 (red)\n");
//             cam->saveImage = true;
//           }
          // manual control flag (from bridge)
          if (bot->joy->manual)
          { // do nothing here - mission is paused (or not started)
  //           printf("Mission:: manual control\n");
            usleep(20000);
          }
          else if (started)
          { // resuming paused mission
            state = mission;
            printf("Mission:: resuming mission %d (until %d)\n", mission, toMission);
          }
          if (bot->event->eventSet(33))
          { // start mission (button pressed)
            printf("Mission:: starting auto mission part from %d to %d\n", fromMission, toMission);
            mission = fromMission;
            state = mission;
            started = true;
            wait4running = true;
            t.now();
            printf("%ld.%06ld Mission %d started\n", t.getSec(), t.getMilisec(), mission);
          }
          else
          { // nothing to do
            if (loop %500 == 0)
            {
              t.now();
              printf("%ld.%06ld Mission: in auto mode, but waiting for start signal (event 33)\n", t.getSec(), t.getMilisec());
            }
            usleep(20000);
          }
          break;
        case 1: // running auto mission
//           printf("Mission:: starting mission 1, state=%d, running=%d, line=%d, thread=%d\n", 
//                  missionState, bot->info.missionRunning, bot->info.missionLineNum, bot->info.missionThread);
          ended = setRacing(missionState);
          if (wait4running and not running and bot->info->missionRunning)
            running = true;
          break;

          case 2:
            ended= setNormal(missionState);
            break;
      
          
        default:
          finished = true;
          break;
      }
      if (ended)
      { // mission part ended
        printf("Mission:: ended\n");
        mission++;
        state++;
        ended = false;
        missionState = 0;
        missionStateOld = -1;
      }
      // debug print
      if (missionState != missionStateOld)
      { // debug print of mission state
//         printf("Mission %d state %d\n", mission, missionState);
        missionStateOld = missionState;
      }
    }
    // debug print end
    // release CPU a bit (10ms)
    usleep(10000);
    // are we finished
    if (bot->event->eventSet(0))// or (running and not bot->info.missionRunning))
    { // robot say stop
      finished = true;
      printf("Mission:: insist we are finished\n");
    }
    else if (mission > toMission)
    { // stop robot
      // make an event 0
      bot->send("stop\n");
      // stop mission loop
      finished = true;
    }
  }
 // system("espeak \"mission ended\" -ven+f4 -s130 -a20 &"); 
  printf("Mission:: all finished\n");
  bot->send("stop\n");
  finished = true;
}


////////////////////////////////////////////////////////////

/**
 * Run mission
 * \param state is kept by caller, but is changed here
 *              therefore defined as reference with the '&'.
 *              State will be 0 at first call.
 * \returns true, when finished. */
bool UMission::mission1(int & state)
{
  bool finished = false;
  // First commands to send to robobot in given mission
  // (robot sends event 1 after driving 1 meter)):
  //
//   float startDist = bot->dist;
  // Primary loop for robobot mission:
  // run the desired mission
  switch (state)
  {
    case 0: // first PART 
      snprintf(lines[0], MAX_LEN, "vel=0.3,acc=2:dist=0.6");
      snprintf(lines[1], MAX_LEN, "tr=0.2:turn=90,time=10");
      // last line should never end, as robot then think we are finished
      // so therefore a timeout of 1 second, to allow next set of
      // commands to be delivered
      snprintf(lines[2], MAX_LEN, "event=1:time=1.1");
      missionSendAndRun(lineList, 3);
      state++;
      break;
    case 1:
      if (bot->event->eventSet(1))
      { // finished first drive
        state = 10;
//         printf("mission finished first part\n");
      }
      break;
    case 10: // go back to start position and stop
      snprintf(lines[0], MAX_LEN, ": dist=0.6");
      snprintf(lines[1], MAX_LEN, "tr=0.05:turn=90,time=10");
      snprintf(lines[2], MAX_LEN, "event=1:time=1.1");
      missionSendAndRun(lineList, 3);
      state++;        
      break;
    case 11:
      if (bot->event->eventSet(1))
      { // finished
        state = 20;
        printf("mission ended\n");
      }
      break;
    case 999:
    default:
      finished = true;
      break;
  }
  return finished;
}

/**
 * Run mission
 * \param state is kept by caller, but is changed here
 *              therefore defined as reference with the '&'.
 *              State will be 0 at first call.
 * \returns true, when finished. */
bool UMission::mission3(int & state)
{
  bool finished = false;
  // First commands to send to robobot in given mission
  // (robot sends event 1 after driving 1 meter)):
  //
//   float startDist = bot->dist;
  // Primary loop for robobot mission:
  // run the desired mission
  switch (state)
  {
    case 0: // Gullutione port + ramp up  
       printf("running mission part 3\n"); 
      snprintf(lines[0], MAX_LEN, "vel=0.4, acc=-1.0, edgel=2.0, white=1: xl>16");
      // last line should never end, as robot then think we are finished
      // so therefore a timeout of 1 second, to allow next set of
      // commands to be delivered
      snprintf(lines[1], MAX_LEN, "event=1:time=1.1");
      missionSendAndRun(lineList, 2);
      state++;
      break;
    case 1: //Gullutione port + ramp up 
      if (bot->event->eventSet(1))
      { // finished first drive
        state = 10;
//         printf("mission finished first part\n");
      }
      break;
    case 10: // Tilt ramp
      snprintf(lines[0], MAX_LEN, "tr=0.125,vel=0.5,acc=2:turn=90,time=10");
      snprintf(lines[1], MAX_LEN, "vel=0.3, acc=-1.0, edgel=0.0, white=1:dist=1");
	  snprintf(lines[2], MAX_LEN, "vel=0.1,acc=10,edgel=0,white=1:lv=0");
	  snprintf(lines[3], MAX_LEN, "vel=0,acc=100:time=2");
      snprintf(lines[4], MAX_LEN, "event=1:time=1.1");
      missionSendAndRun(lineList, 5);
      state++;        
      break;
    case 11:// Tilt ramp
      if (bot->event->eventSet(1))
      { // finished
        state = 20;
        printf("mission ended\n");
      }
      break;
	case 20: // find white line f�r ramp down
		snprintf(lines[0], MAX_LEN, "tr=0.8,vel=0.5,acc=2:turn=-90");
		snprintf(lines[1], MAX_LEN, "vel=0.3,acc=1,edgel=0,white=1:xl>16");
		snprintf(lines[2], MAX_LEN, "vel=0,acc=100:time=2");
		snprintf(lines[3], MAX_LEN, "tr=0.05,vel=0.4,acc=1:turn=-90");
		snprintf(lines[4], MAX_LEN, "event=1:time=1.1");
		missionSendAndRun(lineList, 5);
		state++;
		break;
	case 21:// find white line f�r ramp down
		if (bot->event->eventSet(1))
		{ // finished
			state = 30;
			printf("mission ended\n");
		}
		break;
	case 30: //  ramp down k�r op af 
		snprintf(lines[0], MAX_LEN, "vel=0.6,acc=10,edger=1.0:dist=2.9");
		snprintf(lines[1], MAX_LEN, "vel=0,acc=100:time=1");
		snprintf(lines[2], MAX_LEN, "vel=0.4,acc=2,edger=1.0,white=1:xl>6");
		snprintf(lines[3], MAX_LEN, "event=1:time=1.1");
		missionSendAndRun(lineList, 4);
		state++;
		break;
	case 31://  ramp down k�r op af
		if (bot->event->eventSet(1))
		{ // finished
			state = 40;
			printf("mission ended\n");
		}
		break;
	case 40: // trappe ned
		snprintf(lines[0], MAX_LEN, "vel=0.2,acc=100:time=2,dist=0.23");
		snprintf(lines[1], MAX_LEN, "tr=-0.01,vel=0.5,acc=2:turn=-180");
		snprintf(lines[2], MAX_LEN, "vel=0.3,acc=1,edgel=2.0,white=1:dist=0.2");
		snprintf(lines[3], MAX_LEN, "vel=0.1,acc=5,edgel=0.0,white=1:dist=2");
		snprintf(lines[4], MAX_LEN, "vel=0,acc=100:time=2");
		snprintf(lines[5], MAX_LEN, "vel=0.1,acc=10,edgel=0,white=1:lv=0");
		snprintf(lines[6], MAX_LEN, "event=1:time=1.1");
		missionSendAndRun(lineList, 7);
		state++;
		break;
	case 41:// trappe ned 
		if (bot->event->eventSet(1))
		{ // finished
			state = 50;
			printf("mission ended\n");
		}
		break;
	case 50: // goal
		snprintf(lines[0], MAX_LEN, "tr=0.12,vel=0.4,acc=1:turn=-90");
		snprintf(lines[1], MAX_LEN, "vel=0.3,acc=1,edgel=0,white=1:xl>12");
		snprintf(lines[2], MAX_LEN, "vel=0,acc=100:time=2");
		snprintf(lines[3], MAX_LEN, "tr=0.00,vel=0.4,acc=1:turn=90");
		snprintf(lines[4], MAX_LEN, "vel=0.3,acc=1,edgel=0,white=1:xl>16");
		snprintf(lines[5], MAX_LEN, "event=1:time=1.1");
		missionSendAndRun(lineList, 6);
		state++;
		break;
	case 51:// goal 
		if (bot->event->eventSet(1))
		{ // finished
			state = 999;
			printf("mission ended\n");
		}
		break;
    case 999:
    default:
      finished = true;
      break;
  }
  return finished;
}









bool UMission::setRacing(int & state)
{
  bool finished = false;

  switch (state)
  {
    case 0: // first PART 
    bot->send("cedg 1 0.02 0 1 1e+06 1 1 0.4 0.1 0 1 1 0 1 1 0 1 1e+06 1 0 1 0 1 1 0 1e+06\n"); //set racing line-parameters
  
      state=999;
      break;
   
    case 999:
    default:
      finished = true;
      break;
  }
  return finished;
}

bool UMission::setNormal(int & state)
{
  bool finished = false;

  switch (state)
  {
    case 0: // first PART 
    bot->send("cedg 1 0.08 0 1 1e+06 1 1 0.4 0.1 0 1 1 0 1 1 0 1 1e+06 1 0 1 0 1 1 0 1e+06\n"); //set normal line-parameters
  
      state=999;
      break;
    case 999:
    default:
      finished = true;
      break;
  }
  return finished;
}

