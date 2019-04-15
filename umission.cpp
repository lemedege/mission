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
for (int i = 0; i < missionLineMax; i++)
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


void UMission::run()
{
while (not active and not th1stop)
    usleep(100000);
printf("UMission::run:  active=%d, th1stop=%d\n", active, th1stop);
if (not th1stop)
    runMission();
printf("UMission::run: mission thread ended\n");
}


/**
* Initializes the communication with the robobot_bridge and the REGBOT.
* It further initializes a (maximum) number of mission lines 
* in the REGBOT microprocessor. */
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
// otherwise first samples will produce "false" positive (too short/negative).
bot->send("robot <add irsensor=1,vel=0:dist=0.2\n");
//
// alternating threads (100 and 101, alternating on event 30 and 31 (last 2 events)
bot->send("robot <add thread=100,event=30 : event=31\n");
for (int i = 0; i < missionLineMax; i++)
    // send placeholder lines, that will never finish
    // are to be replaced with real mission
    // NB - hereafter no lines can be added to these threads, just modified
    bot->send("robot <add vel=0 : time=0.1\n");
//
bot->send("robot <add thread=101,event=31 : event=30\n");
for (int i = 0; i < missionLineMax; i++)
    // send placeholder lines, that will never finish
    bot->send("robot <add vel=0 : time=0.1\n");
usleep(10000);
//
printf("UMission::missionInit: base mission loaded\n");
//
//
//
// send subscribe to bridge
bot->pose->subscribe();
//  bot->pose->openLog();       // <--- NB uses diskspace
bot->edge->subscribe();
bot->motor->subscribe();
bot->event->subscribe();
//  bot->event->openLog();       // <--- NB uses diskspace
bot->joy->subscribe();
bot->motor->subscribe();
bot->info->subscribe();
bot->irdist->subscribe();
//  bot->irdist->openLog();       // <--- NB uses diskspace
bot->accgyro->subscribe();
//  bot->accgyro->openLog();       // <--- NB uses diskspace
    // open log in bridge with communication to and 
    // from the REGBOT. 
    // Log should be in robobot_bridge/build
//   bot->send("robot clogopen\n"); // <--- NB uses diskspace
}


void UMission::missionSendAndRun(const char ** missionLines, int missionLineCnt)
{
// Calling missionSendAndRun automatically toggles between thread 100 and 101. 
// Modifies the currently inactive thread and then makes it active. 
const int MSL = 100;
char s[MSL];
int threadToMod = 101;
int startEvent = 31;
//   bot->event->clearEvents();
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
bool ended = false;
bool inManuel = false;
int loop = 0;
int missionState = 0;
// initialize robot mission to do nothing (wait for mission lines)
missionInit();
bot->send("start\n"); // ask robot to start controlled run (ready to execute)
while (not finished and not th1stop)
{ // stay in this mission loop until finished
    loop++;
    if (bot->joy->manual)
    { // just wait, do not continue mission
    usleep(20000);
    if (not inManuel)
        system("espeak \"Mission paused.\" -ven+f4 -s130 -a60 2>/dev/null &"); 
    inManuel = true;
    }
    else
    { // in auto mode
    if (inManuel)
    { // just entered auto mode, so tell.
        inManuel = false;
        system("espeak \"Mission resuming.\" -ven+f4 -s130 -a60 2>/dev/null &"); 
    }
    switch(state)
    {
        case 0: // waiting for start button or manual control
        // (button 0=green, 1=red, blue=2, yellow=3, start=7, back=6, LB=4, RB=5
        if (bot->event->eventSet(33))
        { // start mission (button pressed)
            printf("Mission:: starting auto mission part from %d to %d\n", fromMission, toMission);
            mission = fromMission;
            state = mission;
        }
        break;
        case 1:
            bot->send("robot cedg 1 0.08 0 1 1e+06 1 1 0.4 0.1 0 1 1 0 1 1 0 1 1e+06 1 0 1 0 1 1 0 1e+06\n\0"); //set normal line-parameters
	//bot->send("eew\n\0");
                    ended = true;
                    break;
        case 2: // running auto mission
            ended = racetrack(missionState);
        break;
        
        case 3:
            bot->send("robot cedg 1 0.02 0 1 1e+06 1 1 0.4 0.1 0 1 1 0 1 1 0 1 1e+06 1 0 1 0 1 1 0 1e+06\n\0"); //set racing line-parameters
	//bot->send("eew\n\0");            
ended = true;
            break;
            
            case 4:
            ended= racetrack2(missionState);
            break;
            
            case 5:
                    bot->send("robot cedg 1 0.08 0 1 1e+06 1 1 0.4 0.1 0 1 1 0 1 1 0 1 1e+06 1 0 1 0 1 1 0 1e+06\n\0"); //set normal line-parameter
	//bot->send("eew\n\0");                   
 ended = true;
    break;
			case 6:
				ended = axeGate(missionState);
			break;



        
        default:
        finished = true;
        break;
    }
    if (ended)
    { // start next mission part
        mission++;
        state++;
        ended = false;
        missionState = 0;
    }
    }
    // debug print end
    // release CPU a bit (10ms)
    usleep(10000);
    if (bot->joy->button[1])
    { // red button -> save image
    printf("UMission::runMission:: button 1 (red)\n");
    cam->saveImage = true;
    }
    // are we finished - send event 0 to disable motors
    if (bot->event->eventSet(0))
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
bot->send("stop\n");
const int MSL = 120;
char s[MSL];
//snprintf(s, MSL, "espeak \"robot %s is happy. Don't kill me.\"  -ven+f4 -s130 -a30  2>/dev/null &", bot->info->robotname);
system(s); 
printf("Mission:: all finished\n");
}


////////////////////////////////////////////////////////////

/**
* Run mission 1
* \param state is kept by caller, but is changed here
*              therefore defined as reference with the '&'.
*              State will be 0 at first call.
* \returns true, when finished. */
bool UMission::mission1(int & state) //ikke pille ved den standart mission
{
bool finished = false;
// First commands to send to robobot in given mission
// (robot sends event 1 after driving 1 meter)):
switch (state)
{
    case 0:
    system("espeak \"trigger distance sensor 2 to start\" -ven+f4 -s130 -a60 2>/dev/null &"); 
    printf("\n********************************** espeak: 'trigger IR2 to start'\n");
    state++;
    break;
    case 1: // first PART 
    snprintf(lines[0], MAX_LEN, "vel=0 : ir2 < 0.3");
    snprintf(lines[1], MAX_LEN, "vel=0.3,acc=2:dist=0.6");
    snprintf(lines[2], MAX_LEN, "tr=0.2:turn=90,time=10");
    // last line should never end, as robot then think we are finished
    // so therefore a timeout of 1 second, to allow next set of
    // commands to be delivered
    snprintf(lines[3], MAX_LEN, "event=1:time=1.1");
    snprintf(lines[4], MAX_LEN, "vel=0: dist=0.5");
    missionSendAndRun(lineList, 5);
    // make sure event 1 is cleared
    bot->event->eventSet(1);
    state++;
    break;
    case 2:
    // wait for event 1
    if (bot->event->eventSet(1))
    { // finished first drive
        state = 10;
        system("espeak \"Sending code snippet 2.\" -ven+f4 -s130 -a60 2>/dev/null &"); 
    }
    break;
    case 10: // go back to start position and stop
    snprintf(lines[0], MAX_LEN, "vel=0.3 : dist=0.6");
    snprintf(lines[1], MAX_LEN, "tr=0.05:turn=90,time=10");
    snprintf(lines[2], MAX_LEN, "event=2:time=1.1");
    snprintf(lines[3], MAX_LEN, "vel=0: dist=0.5");
    missionSendAndRun(lineList, 4);
    // make sure event 2 is cleared
    bot->event->eventSet(2);
    state++;        
    break;
    case 11:
    // wait for event 2
    if (bot->event->eventSet(2))
    { // finished
        state = 20;
        printf("UMission::mission1: mission ended \n");
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
* Run mission 2
*/
bool UMission::mission2(int & state)
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
    case 0: // Gullutione port
    printf("running mission part 3\n"); 
    snprintf(lines[0], MAX_LEN, "vel=0.4, acc=1.0, edger=2.0, white=1: dist=60");
    // last line should never end, as robot then think we are finished
    // so therefore a timeout of 1 second, to allow next set of
    // commands to be delivered
    snprintf(lines[1], MAX_LEN, "event=1:time=1.1");
    missionSendAndRun(lineList, 2);
    state++;
    break;
    case 1:  
    if (bot->event->eventSet(1))
    { // finished first drive
        state = 999;
//         printf("mission finished first part\n");
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
* Run mission 3
*/
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
    case 0: // Gullutione port
    printf("running mission part 3\n"); 
    snprintf(lines[0], MAX_LEN, "vel=0.4, acc=1.0, edgel=0.0, white=1: dist=6");
	snprintf(lines[1], MAX_LEN, "vel=0.4, acc=-1.0, edgel=2.0, white=1: xl>16");
    // last line should never end, as robot then think we are finished
    // so therefore a timeout of 1 second, to allow next set of
    // commands to be delivered
    snprintf(lines[2], MAX_LEN, "event=1:time=1.1");
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
    case 10: // ramp up and Tilt ramp
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
        case 20: // find white line f???r ramp down
                snprintf(lines[0], MAX_LEN, "tr=0.8,vel=0.5,acc=2:turn=-90");
                snprintf(lines[1], MAX_LEN, "vel=0.3,acc=1,edgel=0,white=1:xl>16");
                snprintf(lines[2], MAX_LEN, "vel=0.3, acc=-1.0, edgel=0.0, white=1: dist=0.2");
                snprintf(lines[3], MAX_LEN, "vel=0.3,acc=1,edgel=0,white=1:xl>16");
				snprintf(lines[4], MAX_LEN, "vel=0,acc=100:time=2");
                snprintf(lines[5], MAX_LEN, "event=1:time=1.1");
                missionSendAndRun(lineList, 6);
                state++;
                break;
        case 21:// find white line f???r ramp down
                if (bot->event->eventSet(1))
                { // finished
                        state = 30;
                        printf("mission ended\n");
                }
                break;
        case 30: //  ramp down k???r op af 
                snprintf(lines[0], MAX_LEN, "tr=0.05,vel=0.4,acc=1:turn=-90");
                snprintf(lines[1], MAX_LEN, "vel=0.55,acc=10,edger=1.0:dist=2.9");
                snprintf(lines[2], MAX_LEN, "vel=0.2,acc=2,edger=1.0,white=1:xl>4");
                snprintf(lines[3], MAX_LEN, "event=1:time=1.1");
                missionSendAndRun(lineList, 4);
                state++;
                break;
        case 31://  ramp down k???r op af
                if (bot->event->eventSet(1))
                { // finished
                        state = 40;
                        printf("mission ended\n");
                }
                break;
        case 40: // trappe ned
                snprintf(lines[0], MAX_LEN, "vel=0.2,acc=100:time=2,dist=0.23");
                snprintf(lines[1], MAX_LEN, "tr=-0.01,vel=0.5,acc=2:turn=-180");
                snprintf(lines[2], MAX_LEN, "vel=0.2,acc=100,edger=1.0,white=1:dist=0.4");
                snprintf(lines[3], MAX_LEN, "vel=0.2,acc=1,edgel=0.0,white=1:dist=1.2");
                snprintf(lines[4], MAX_LEN, "vel=0.1,acc=5,edgel=0.0,white=1:xl>5");
                snprintf(lines[5], MAX_LEN, "vel=0,acc=100:time=2");
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
                snprintf(lines[0], MAX_LEN, "vel=0.3,acc=1,edgel=0,white=1:xl>16");
                snprintf(lines[1], MAX_LEN, "vel=0,acc=100:time=2");
                snprintf(lines[2], MAX_LEN, "tr=0.00,vel=0.4,acc=1:turn=-90");
                snprintf(lines[3], MAX_LEN, "vel=0.3,acc=1,edgel=0,white=1:dist=4.0");
                snprintf(lines[4], MAX_LEN, "stop");
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

/**
* Run mission 4
*/
bool UMission::mission4(int & state)
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
    case 0: // Gullutione port
    printf("running mission part 3\n"); 
    snprintf(lines[0], MAX_LEN, "vel=0.4, acc=1.0, edgel=0.0, white=1: dist=6");
	snprintf(lines[1], MAX_LEN, "vel=0.4, acc=-1.0, edgel=2.0, white=1: xl>16");
    // last line should never end, as robot then think we are finished
    // so therefore a timeout of 1 second, to allow next set of
    // commands to be delivered
    snprintf(lines[2], MAX_LEN, "event=1:time=1.1");
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
    case 10: // ramp up and Tilt ramp
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
        case 20: // find white line f???r ramp down
                snprintf(lines[0], MAX_LEN, "tr=0.8,vel=0.5,acc=2:turn=-90");
                snprintf(lines[1], MAX_LEN, "vel=0.3,acc=1,edgel=0,white=1:xl>16");
                snprintf(lines[2], MAX_LEN, "vel=0.3, acc=-1.0, edgel=0.0, white=1: dist=0.2");
                snprintf(lines[3], MAX_LEN, "vel=0.3,acc=1,edgel=0,white=1:xl>16");
				snprintf(lines[4], MAX_LEN, "vel=0,acc=100:time=2");
                snprintf(lines[5], MAX_LEN, "event=1:time=1.1");
                missionSendAndRun(lineList, 6);
                state++;
                break;
        case 21:// find white line f???r ramp down
                if (bot->event->eventSet(1))
                { // finished
                        state = 30;
                        printf("mission ended\n");
                }
                break;
        case 30: //  ramp down k???r op af 
                snprintf(lines[0], MAX_LEN, "tr=0.05,vel=0.4,acc=1:turn=-90");
                snprintf(lines[1], MAX_LEN, "vel=0.55,acc=10,edger=1.0:dist=2.9");
                snprintf(lines[2], MAX_LEN, "vel=0.2,acc=2,edger=1.0,white=1:xl>4");
                snprintf(lines[3], MAX_LEN, "event=1:time=1.1");
                missionSendAndRun(lineList, 4);
                state++;
                break;
        case 31://  ramp down k???r op af
                if (bot->event->eventSet(1))
                { // finished
                        state = 40;
                        printf("mission ended\n");
                }
                break;
        case 40: // trappe ned
                snprintf(lines[0], MAX_LEN, "vel=0.2,acc=100:time=2,dist=0.23");
                snprintf(lines[1], MAX_LEN, "tr=-0.01,vel=0.5,acc=2:turn=-180");
                snprintf(lines[2], MAX_LEN, "vel=0.2,acc=100,edger=1.0,white=1:dist=0.4");
                snprintf(lines[3], MAX_LEN, "vel=0.2,acc=1,edgel=0.0,white=1:dist=1.2");
                snprintf(lines[4], MAX_LEN, "vel=0.1,acc=5,edgel=0.0,white=1:xl>5");
                snprintf(lines[5], MAX_LEN, "vel=0,acc=100:time=2");
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
        case 50: //trappe til økseport del 1
                snprintf(lines[0], MAX_LEN, "vel=0.3,acc=1,edgel=0,white=1:xl>16");
                snprintf(lines[1], MAX_LEN, "vel=0,acc=100:time=2");
                snprintf(lines[2], MAX_LEN, "tr=0.00,vel=0.4,acc=1:turn=-90");
                snprintf(lines[3], MAX_LEN, "vel=0.3,acc=2,edgel=0.0, white=1 : dist=0.1");
                snprintf(lines[4], MAX_LEN, "vel=0.35, acc=2.0, edger=0.0, white=1:xl>16");
				snprintf(lines[5], MAX_LEN, "vel=0.0, acc=100.0:time=0.1");
				snprintf(lines[6], MAX_LEN, "tr=0.0,vel=0.3,acc=2:turn=90");
				snprintf(lines[7], MAX_LEN, "vel=0.4,acc=2,edgel=0.0, white=1 : dist=0.3");
				snprintf(lines[8], MAX_LEN, "vel=0.0,acc=100.0:time=0.1");
                snprintf(lines[9], MAX_LEN, "event=1:time=1.1");
                missionSendAndRun(lineList, 10);
                state++;
                break;
        case 51:// goal 
                if (bot->event->eventSet(1))
                { // finished
                        state = 60;
                        printf("mission ended\n");
                }
                break;
				
		case 60: // foran økseport + kør igennem stil ved regbot 
                snprintf(lines[0], MAX_LEN, "vel=0.0,acc=100.0:ir2<0.21");
                snprintf(lines[1], MAX_LEN, "vel=0.0,acc=5:ir2>0.4");
                snprintf(lines[2], MAX_LEN, "vel=0.0,acc=5:time=0.1");
                snprintf(lines[3], MAX_LEN, "vel=0.5,acc=5, edgel=0.0, white=1:dist=0.5");
                snprintf(lines[4], MAX_LEN, "vel=0.0, acc=100.0, edgel=0.0, white=1:time=0.1");
				snprintf(lines[5], MAX_LEN, "tr=0.1,vel=0.3,acc=2:turn=90");
				snprintf(lines[6], MAX_LEN, "vel=0.5,acc=5:dist=0.55");
				snprintf(lines[7], MAX_LEN, "vel=0.0, acc=100.0:time=0.1");
                snprintf(lines[8], MAX_LEN, "event=1:time=1.1");
                missionSendAndRun(lineList, 9);
                state++;
                break;
        case 61:// goal 
                if (bot->event->eventSet(1))
                { // finished
                        state = 70;
                        printf("mission ended\n");
                }
                break;
				
		case 70: // foran regbot 
                snprintf(lines[0], MAX_LEN, "vel=0.0,acc=100.0:ir2<0.4");
                snprintf(lines[1], MAX_LEN, "vel=0.0,acc=5.0:ir2>0.6");
                snprintf(lines[2], MAX_LEN, "vel=0.0,acc=5:time=0.5");
                snprintf(lines[3], MAX_LEN, "vel=0.35,acc=5:lv=1");
                snprintf(lines[4], MAX_LEN, "tr=0.0,vel=0.5,acc=2:turn=90");
				snprintf(lines[5], MAX_LEN, "vel=0.35,acc=5, edgel=0.0, white=1:xl>16");
				snprintf(lines[6], MAX_LEN, "vel=0.35, acc=5, edger=0.0, white=1: dist=0.2");
				snprintf(lines[7], MAX_LEN, "vel=0.35,acc=5, edgel=0.0, white=1:xl>16");
                snprintf(lines[8], MAX_LEN, "event=1:time=1.1");
                missionSendAndRun(lineList, 9);
                state++;
                break;
        case 71:// goal 
                if (bot->event->eventSet(1))
                { // finished
                        state = 80;
                        printf("mission ended\n");
                }
                break;
				
		case 80: // rundt i regbot bane cirkel bla  
                snprintf(lines[0], MAX_LEN, "vel=0.35, acc=5, edger=0.0, white=1: dist=0.2");
                snprintf(lines[1], MAX_LEN, "vel=0.35,acc=5, edgel=0.0, white=1:xl>16");
                snprintf(lines[2], MAX_LEN, "vel=0.35, acc=5, edger=0.0, white=1: dist=0.2");
                snprintf(lines[3], MAX_LEN, "vel=0.35,acc=5, edgel=0.0, white=1:xl>16");
                snprintf(lines[4], MAX_LEN, "vel=0.35, acc=5, edger=0.0, white=1: dist=0.2");
				snprintf(lines[5], MAX_LEN, "vel=0.35,acc=5, edgel=0.0, white=1:xl>16");
				snprintf(lines[6], MAX_LEN, "tr=0.0,vel=0.3,acc=2:turn=-90");
				snprintf(lines[7], MAX_LEN, "vel=0.3, acc=2.0, edger=0.0, white=1:dist=0.7");
				snprintf(lines[8], MAX_LEN, "tr=0.0,vel=0.3,acc=2:turn=-180");
				snprintf(lines[9], MAX_LEN, "vel=0.0, acc=100.0, edgel=0.0, white=1:time=0.1");
                snprintf(lines[10], MAX_LEN, "event=1:time=1.1");
                missionSendAndRun(lineList, 11);
                state++;
                break;
        case 81:// goal 
                if (bot->event->eventSet(1))
                { // finished
                        state = 90;
                        printf("mission ended\n");
                }
                break;
				
		case 90: // hjem til goal  
                snprintf(lines[0], MAX_LEN, "vel=0.55, acc=3.0, edger=1.0, white=1: dist=0.35");
                snprintf(lines[1], MAX_LEN, "vel=0.0, acc=100.0, edgel=0.0, white=1:time=0.2");
                snprintf(lines[2], MAX_LEN, "vel=0.0,acc=100.0, edgel=0.0, white=1:ir2<0.4");
                snprintf(lines[3], MAX_LEN, "vel=0.0,acc=5.0, edgel=0.0, white=1:ir2>0.6");
                snprintf(lines[4], MAX_LEN, "vel=0.35,acc=5, edgel=0.0, white=1:xl>16");
				snprintf(lines[5], MAX_LEN, "tr=0.0,vel=0.3,acc=2:turn=-90");
				snprintf(lines[6], MAX_LEN, "vel=0.35,acc=5, edgel=0.0, white=1:xl>16");
				snprintf(lines[7], MAX_LEN, "vel=0.35,acc=5, edgel=0.0, white=1:xl>16");
				snprintf(lines[8], MAX_LEN, "tr=0.0,vel=0.3,acc=2:turn=90");
                snprintf(lines[9], MAX_LEN, "event=1:time=1.1");
                missionSendAndRun(lineList, 10);
                state++;
                break;
        case 91:// goal 
                if (bot->event->eventSet(1))
                { // finished
                        state = 100;
                        printf("mission ended\n");
                }
                break;
				
		case 100: // hjem til goal  
                snprintf(lines[0], MAX_LEN, "vel=0.55, acc=3.0, edger=1.0, white=1: dist=4");
                snprintf(lines[1], MAX_LEN, "event=1:time=1.1");
                missionSendAndRun(lineList, 2);
                state++;
                break;
        case 101:// goal 
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

/**
* Run mission 5
*/
bool UMission::mission5(int & state)
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
    case 0: // Gullutione port
    printf("running mission part 3\n"); 
    snprintf(lines[0], MAX_LEN, "vel=0.4, acc=1.0, edgel=0.0, white=1: dist=6");
	snprintf(lines[1], MAX_LEN, "vel=0.4, acc=-1.0, edgel=2.0, white=1: xl>16");
    // last line should never end, as robot then think we are finished
    // so therefore a timeout of 1 second, to allow next set of
    // commands to be delivered
    snprintf(lines[2], MAX_LEN, "event=1:time=1.1");
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
    case 10: // ramp up and Tilt ramp
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
        case 20: // find white line f???r ramp down
                snprintf(lines[0], MAX_LEN, "tr=0.8,vel=0.5,acc=2:turn=-90");
                snprintf(lines[1], MAX_LEN, "vel=0.3,acc=1,edgel=0,white=1:xl>16");
                snprintf(lines[2], MAX_LEN, "vel=0.3, acc=-1.0, edgel=0.0, white=1: dist=0.2");
                snprintf(lines[3], MAX_LEN, "vel=0.3,acc=1,edgel=0,white=1:xl>16");
				snprintf(lines[4], MAX_LEN, "vel=0,acc=100:time=2");
                snprintf(lines[5], MAX_LEN, "event=1:time=1.1");
                missionSendAndRun(lineList, 6);
                state++;
                break;
        case 21:// find white line f???r ramp down
                if (bot->event->eventSet(1))
                { // finished
                        state = 30;
                        printf("mission ended\n");
                }
                break;
        case 30: //  ramp down k???r op af 
                snprintf(lines[0], MAX_LEN, "tr=0.05,vel=0.4,acc=1:turn=-90");
                snprintf(lines[1], MAX_LEN, "vel=0.55,acc=10,edger=1.0:dist=2.9");
                snprintf(lines[2], MAX_LEN, "vel=0.2,acc=2,edger=1.0,white=1:xl>4");
                snprintf(lines[3], MAX_LEN, "event=1:time=1.1");
                missionSendAndRun(lineList, 4);
                state++;
                break;
        case 31://  ramp down k???r op af
                if (bot->event->eventSet(1))
                { // finished
                        state = 40;
                        printf("mission ended\n");
                }
                break;
        case 40: // trappe ned
                snprintf(lines[0], MAX_LEN, "vel=0.2,acc=100:time=2,dist=0.23");
                snprintf(lines[1], MAX_LEN, "tr=-0.01,vel=0.5,acc=2:turn=-180");
                snprintf(lines[2], MAX_LEN, "vel=0.2,acc=100,edger=1.0,white=1:dist=0.4");
                snprintf(lines[3], MAX_LEN, "vel=0.2,acc=1,edgel=0.0,white=1:dist=1.2");
                snprintf(lines[4], MAX_LEN, "vel=0.1,acc=5,edgel=0.0,white=1:xl>5");
                snprintf(lines[5], MAX_LEN, "vel=0,acc=100:time=2");
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
        case 50: //trappe til økseport del 1
                snprintf(lines[0], MAX_LEN, "vel=0.3,acc=1,edgel=0,white=1:xl>16");
                snprintf(lines[1], MAX_LEN, "vel=0,acc=100:time=2");
                snprintf(lines[2], MAX_LEN, "tr=0.00,vel=0.4,acc=1:turn=-90");
                snprintf(lines[3], MAX_LEN, "vel=0.3,acc=2,edgel=0.0, white=1 : dist=0.1");
                snprintf(lines[4], MAX_LEN, "vel=0.35, acc=2.0, edger=0.0, white=1:xl>16");
				snprintf(lines[5], MAX_LEN, "vel=0.0, acc=100.0:time=0.1");
				snprintf(lines[6], MAX_LEN, "tr=0.0,vel=0.3,acc=2:turn=90");
				snprintf(lines[7], MAX_LEN, "vel=0.4,acc=2,edgel=0.0, white=1 : dist=0.3");
				snprintf(lines[8], MAX_LEN, "vel=0.0,acc=100.0:time=0.1");
                snprintf(lines[9], MAX_LEN, "event=1:time=1.1");
                missionSendAndRun(lineList, 10);
                state++;
                break;
        case 51:// goal 
                if (bot->event->eventSet(1))
                { // finished
                        state = 60;
                        printf("mission ended\n");
                }
                break;
				
		case 60: // foran økseport + kør igennem stil ved regbot 
                snprintf(lines[0], MAX_LEN, "vel=0.0,acc=100.0:ir2<0.21");
                snprintf(lines[1], MAX_LEN, "vel=0.0,acc=5:ir2>0.4");
                snprintf(lines[2], MAX_LEN, "vel=0.0,acc=5:time=0.1");
                snprintf(lines[3], MAX_LEN, "vel=0.5,acc=5, edgel=0.0, white=1:dist=0.5");
                snprintf(lines[4], MAX_LEN, "vel=0.0, acc=100.0, edgel=0.0, white=1:time=0.1");
				snprintf(lines[5], MAX_LEN, "tr=0.1,vel=0.3,acc=2:turn=90");
				snprintf(lines[6], MAX_LEN, "vel=0.5,acc=5:dist=0.55");
				snprintf(lines[7], MAX_LEN, "vel=0.0, acc=100.0:time=0.1");
                snprintf(lines[8], MAX_LEN, "event=1:time=1.1");
                missionSendAndRun(lineList, 9);
                state++;
                break;
        case 61:// goal 
                if (bot->event->eventSet(1))
                { // finished
                        state = 70;
                        printf("mission ended\n");
                }
                break;
				
		case 70: // foran regbot 
                snprintf(lines[0], MAX_LEN, "vel=0.0,acc=100.0:ir2<0.4");
                snprintf(lines[1], MAX_LEN, "vel=0.0,acc=5.0:ir2>0.6");
                snprintf(lines[2], MAX_LEN, "vel=0.0,acc=5:time=0.5");
                snprintf(lines[3], MAX_LEN, "vel=0.35,acc=5:lv=1");
                snprintf(lines[4], MAX_LEN, "tr=0.0,vel=0.5,acc=2:turn=90");
				snprintf(lines[5], MAX_LEN, "vel=0.35,acc=5, edgel=0.0, white=1:xl>16");
				snprintf(lines[6], MAX_LEN, "vel=0.35, acc=5, edger=0.0, white=1: dist=0.2");
				snprintf(lines[7], MAX_LEN, "vel=0.35,acc=5, edgel=0.0, white=1:xl>16");
                snprintf(lines[8], MAX_LEN, "event=1:time=1.1");
                missionSendAndRun(lineList, 9);
                state++;
                break;
        case 71:// goal 
                if (bot->event->eventSet(1))
                { // finished
                        state = 80;
                        printf("mission ended\n");
                }
                break;
				
		case 80: // rundt i regbot bane cirkel bla  
                snprintf(lines[0], MAX_LEN, "vel=0.35, acc=5, edger=0.0, white=1: dist=0.2");
                snprintf(lines[1], MAX_LEN, "vel=0.35,acc=5, edgel=0.0, white=1:xl>16");
                snprintf(lines[2], MAX_LEN, "vel=0.35, acc=5, edger=0.0, white=1: dist=0.2");
                snprintf(lines[3], MAX_LEN, "vel=0.35,acc=5, edgel=0.0, white=1:xl>16");
                snprintf(lines[4], MAX_LEN, "vel=0.35, acc=5, edger=0.0, white=1: dist=0.2");
				snprintf(lines[5], MAX_LEN, "vel=0.35,acc=5, edgel=0.0, white=1:xl>16");
				snprintf(lines[6], MAX_LEN, "tr=0.0,vel=0.3,acc=2:turn=-90");
				snprintf(lines[7], MAX_LEN, "vel=0.3, acc=2.0, edger=0.0, white=1:dist=0.7");
				snprintf(lines[8], MAX_LEN, "tr=0.0,vel=0.3,acc=2:turn=-180");
				snprintf(lines[9], MAX_LEN, "vel=0.0, acc=100.0, edgel=0.0, white=1:time=0.1");
                snprintf(lines[10], MAX_LEN, "event=1:time=1.1");
                missionSendAndRun(lineList, 11);
                state++;
                break;
        case 81:// goal 
                if (bot->event->eventSet(1))
                { // finished
                        state = 90;
                        printf("mission ended\n");
                }
                break;
				
		case 90: // op til tunnel  
                snprintf(lines[0], MAX_LEN, "vel=0.55, acc=3.0, edger=1.0, white=1: dist=0.35");
                snprintf(lines[1], MAX_LEN, "vel=0.0, acc=100.0, edgel=0.0, white=1:time=0.2");
                snprintf(lines[2], MAX_LEN, "vel=0.0,acc=100.0, edgel=0.0, white=1:ir2<0.4");
                snprintf(lines[3], MAX_LEN, "vel=0.0,acc=5.0, edgel=0.0, white=1:ir2>0.6");
                snprintf(lines[4], MAX_LEN, "vel=0.0,acc=5, edgel=0.0, white=1:time=0.5");
				snprintf(lines[5], MAX_LEN, "vel=0.35,acc=5, edgel=0.0, white=1:xl>16");
				snprintf(lines[6], MAX_LEN, "vel=0.2,acc=5.0, edgel=0.0, white=1:ir2<0.25");
				snprintf(lines[7], MAX_LEN, "vel=0.0, acc=100.0, edgel=0.0, white=1:time=0.2");
				snprintf(lines[8], MAX_LEN, "event=1:time=1.1");
                missionSendAndRun(lineList, 9);
                state++;
                break;
        case 91:// goal 
                if (bot->event->eventSet(1))
                { // finished
                        state = 100;
                        printf("mission ended\n");
                }
                break;
		
		case 100: // gennem tunnel  
                snprintf(lines[0], MAX_LEN, "tr=0.05,vel=0.3,acc=3:turn=90");
                snprintf(lines[1], MAX_LEN, "vel=0.3,acc=3:dist=0.50");
                snprintf(lines[2], MAX_LEN, "tr=0.05,vel=0.3,acc=3:turn=-90");
                snprintf(lines[3], MAX_LEN, "vel=0.3,acc=3:dist=0.40");
                snprintf(lines[4], MAX_LEN, "tr=0.05,vel=0.3,acc=3:turn=-90");
				snprintf(lines[5], MAX_LEN, "vel=0.2,acc=3,edgel=0.0,white=1:ir2<0.20");
				snprintf(lines[6], MAX_LEN, "tr=0.05,vel=0.3,acc=3:turn=-90");
				snprintf(lines[7], MAX_LEN, "vel=0.3,acc=3:dist=0.20");
				snprintf(lines[8], MAX_LEN, "tr=0.2,vel=0.3,acc=3:turn=90,xl>16");
				snprintf(lines[9], MAX_LEN, "tr=0.2,vel=0.3,acc=3:turn=90");
				snprintf(lines[10], MAX_LEN, "event=1:time=1.1");
                missionSendAndRun(lineList, 11);
                state++;
                break;
        case 101:// goal 
                if (bot->event->eventSet(1))
                { // finished
                        state = 110;
                        printf("mission ended\n");
                }
                break;
				
		case 110: // gennem tunnel + rundt om   
                snprintf(lines[0], MAX_LEN, "vel=0.3,acc=3,edgel=0.0,white=1:dist=0.65");
                snprintf(lines[1], MAX_LEN, "tr=0.2,vel=0.3,acc=3:turn=-90");
                snprintf(lines[2], MAX_LEN, "vel=0.3,acc=3:dist=0.27");
                snprintf(lines[3], MAX_LEN, "tr=0.3,vel=0.3,acc=3:turn=-90");
                snprintf(lines[4], MAX_LEN, "vel=0.5,acc=5:dist=1.05");
				snprintf(lines[5], MAX_LEN, "tr=0.2,vel=0.3,acc=3:turn=-90");
				snprintf(lines[6], MAX_LEN, "vel=0.3,acc=5,white=1:dist=0.2");
				snprintf(lines[7], MAX_LEN, "vel=0.3,acc=5,white=1:xl>18");
				snprintf(lines[8], MAX_LEN, "tr=0.0,vel=0.3,acc=3:turn=-90");
				snprintf(lines[9], MAX_LEN, "vel=0.1,acc=3,edgel=0.0,white=1:time=7.0");
				snprintf(lines[10], MAX_LEN, "event=1:time=1.1");
                missionSendAndRun(lineList, 11);
                state++;
                break;
        case 111:// goal 
                if (bot->event->eventSet(1))
                { // finished
                        state = 120;
                        printf("mission ended\n");
                }
                break;
				
		case 120: // rundt om   luk portene 
                snprintf(lines[0], MAX_LEN, "vel=-0.1,acc=3:dist=0.1");
                snprintf(lines[1], MAX_LEN, "tr=0.05,vel=0.3,acc=3:turn=90");
                snprintf(lines[2], MAX_LEN, "vel=0.3,acc=3:dist=0.50");
                snprintf(lines[3], MAX_LEN, "tr=0.05,vel=0.3,acc=3:turn=-90");
                snprintf(lines[4], MAX_LEN, "vel=0.3,acc=3:dist=0.95");
				snprintf(lines[5], MAX_LEN, "tr=0.2,vel=0.3,acc=3:turn=-90");
				snprintf(lines[6], MAX_LEN, "vel=0.3,acc=5,white=1:xl>18");
				snprintf(lines[7], MAX_LEN, "tr=0.0,vel=0.3,acc=3:turn=-90");
				snprintf(lines[8], MAX_LEN, "vel=0.1,acc=3,edgel=0.0,white=1:time=7.0");
				snprintf(lines[9], MAX_LEN, "vel=-0.1,acc=3:dist=0.1");
				snprintf(lines[10], MAX_LEN, "event=1:time=1.1");
                missionSendAndRun(lineList, 11);
                state++;
                break;
        case 121:// goal 
                if (bot->event->eventSet(1))
                { // finished
                        state = 130;
                        printf("mission ended\n");
                }
                break;
				
		case 130: //luk port gør klar til racebane    
                snprintf(lines[0], MAX_LEN, "tr=0.0,vel=0.3,acc=3:turn=180");
                snprintf(lines[1], MAX_LEN, "vel=-0.2,acc=30:time=4");
                snprintf(lines[2], MAX_LEN, "vel=0.3,acc=3,edger=0.0,white=1:dist=2.28");
                snprintf(lines[3], MAX_LEN, "vel=0.0, acc=100.0, edgel=0.0, white=1:time=0.2");
                snprintf(lines[4], MAX_LEN, "tr=0.05,vel=0.3,acc=3:turn=-24");
				snprintf(lines[5], MAX_LEN, "vel=-0.1,acc=3:dist=0.15");
				snprintf(lines[6], MAX_LEN, "vel=0.0, acc=100.0, edgel=0.0, white=1:time=0.2");
				snprintf(lines[7], MAX_LEN, "event=1:time=1.1");
                missionSendAndRun(lineList, 8);
                state++;
                break;
        case 131:// goal 
                if (bot->event->eventSet(1))
                { // finished
                        state = 140;
                        printf("mission ended\n");
                }
                break;
				
		case 140: // first PART 
			bot->send("cedg 1 0.02 0 1 1e+06 1 1 0.4 0.1 0 1 1 0 1 1 0 1 1e+06 1 0 1 0 1 1 0 1e+06\n\0"); //set racing line-parameters
			snprintf(lines[0], MAX_LEN, "event=1:time=1.1");
			missionSendAndRun(lineList, 1);
			state++;
			break;
		
		case 141	
			if (bot->event->eventSet(1))
			{ // finished first drive
				state = 150;
				printf("mission ended\n");
			}
			break;
			
		case 150: //luk port gør klar til racebane    
                snprintf(lines[0], MAX_LEN, "vel=0.4,acc=4,edger=2.0,white=1:dist=0.2");
                snprintf(lines[1], MAX_LEN, "vel=1.0,acc=4,edgel=0.0,white=1:dist=0.2");
                snprintf(lines[2], MAX_LEN, "vel=2.0,acc=8,edgel=0.0,white=1:xl>8");
                snprintf(lines[3], MAX_LEN, "vel=2.0,acc=5,edgel=0.0,white=1:dist=3");
                snprintf(lines[4], MAX_LEN, "vel=0,acc=1000:time=1");
				snprintf(lines[5], MAX_LEN, "vel=0.3,acc=8,edgel=0.0,white=1:xl>16");
				snprintf(lines[6], MAX_LEN, "vel=0,acc=1000:time=1");
				snprintf(lines[7], MAX_LEN, "event=1:time=1.1");
                missionSendAndRun(lineList, 8);
                state++;
                break;
        case 151:// goal 
                if (bot->event->eventSet(1))
                { // finished
                        state = 160;
                        printf("mission ended\n");
                }
                break;
				
		case 160: // first PART 
			bot->send("cedg 1 0.08 0 1 1e+06 1 1 0.4 0.1 0 1 1 0 1 1 0 1 1e+06 1 0 1 0 1 1 0 1e+06\n\0"); //set normal line-parameters
			snprintf(lines[0], MAX_LEN, "event=1:time=1.1");
			missionSendAndRun(lineList, 1);
			state++;
			break;
		
		case 161	
			if (bot->event->eventSet(1))
			{ // finished first drive
				state = 170;
				printf("mission ended\n");
			}
			break;
			
		case 170: fra racebane til cirkel3     
                snprintf(lines[0], MAX_LEN, "tr=0.0,vel=0.3,acc=3:turn=-90");
                snprintf(lines[1], MAX_LEN, "vel=0.4,acc=4,edger=0.0,white=1:dist=2.3");
                snprintf(lines[2], MAX_LEN, "vel=0,acc=1000:time=1");
                snprintf(lines[3], MAX_LEN, "tr=0.0,vel=0.25,acc=3:turn=180");
                snprintf(lines[4], MAX_LEN, "vel=0,acc=1000:time=1");
				snprintf(lines[5], MAX_LEN, "servo=2,pservo=700:time=1.0");
				snprintf(lines[6], MAX_LEN, "vel=-0.4,acc=6:time=1.5");
				snprintf(lines[7], MAX_LEN, "vel=-0.4,acc=5,servo=2,pservo=-700:time=5,dist=0.33");
				snprintf(lines[8], MAX_LEN, "vel=0,acc=1000:time=0.1");
				snprintf(lines[9], MAX_LEN, "event=1:time=1.1");
                missionSendAndRun(lineList, 10);
                state++;
                break;
        case 171:// goal 
                if (bot->event->eventSet(1))
                { // finished
                        state = 180;
                        printf("mission ended\n");
                }
                break;
				
		case 180: rundt i cirkel3     
                snprintf(lines[0], MAX_LEN, "servo=2,pservo=700:time=0.2");
                snprintf(lines[1], MAX_LEN, "tr=0.0,vel=0.3,acc=3:turn=120");
                snprintf(lines[2], MAX_LEN, "tr=0.32,vel=0.3,acc=3:turn=260");
                snprintf(lines[3], MAX_LEN, "vel=0.4,acc=6:dist=0.6");
                snprintf(lines[4], MAX_LEN, "vel=0.2,acc=4:lv=0");
				snprintf(lines[5], MAX_LEN, "vel=0.3,acc=4,edger=0.0,white=1:xl>4");
				snprintf(lines[6], MAX_LEN, "tr=0.0,vel=0.3,acc=3:turn=90");
				snprintf(lines[7], MAX_LEN, "vel=0.3,acc=4,edger=0.0,white=1:xl>4");
				snprintf(lines[8], MAX_LEN, "vel=0.4,acc=6:dist=0.1");
				snprintf(lines[9], MAX_LEN, "event=1:time=1.1");
                missionSendAndRun(lineList, 10);
                state++;
                break;
        case 181:// goal 
                if (bot->event->eventSet(1))
                { // finished
                        state = 190;
                        printf("mission ended\n");
                }
                break;
				
		case 190: rundt i cirkel3 op til axe gate     
                snprintf(lines[0], MAX_LEN, "vel=0.3,acc=4,edger=0.0,white=1:xl>4");
                snprintf(lines[1], MAX_LEN, "vel=0.4,acc=6:dist=0.1");
                snprintf(lines[2], MAX_LEN, "vel=0.3,acc=4,edger=0.0,white=1:xl>4");
                snprintf(lines[3], MAX_LEN, "tr=0.0,vel=0.3,acc=3:turn=90");
                snprintf(lines[4], MAX_LEN, "vel=0.4,acc=6:dist=0.9");
				snprintf(lines[5], MAX_LEN, "vel=0.3,acc=4,edger=0.0,white=1:xl>4");
				snprintf(lines[6], MAX_LEN, "vel=0,acc=1000:time=0.1");
				snprintf(lines[7], MAX_LEN, "event=1:time=1.1");
                missionSendAndRun(lineList, 8);
                state++;
                break;
        case 191:// goal 
                if (bot->event->eventSet(1))
                { // finished
                        state = 200;
                        printf("mission ended\n");
                }
                break;
				
		case 200: rundt i cirkel3     
                snprintf(lines[0], MAX_LEN, "vel=0.4,acc=2,edgel=0.0, white=1 : ir2<0.2,dist=0.45");
                snprintf(lines[1], MAX_LEN, "vel=0.0,acc=100.0:time=0.1");
                snprintf(lines[2], MAX_LEN, "vel=0.0,acc=100.0:ir2<0.21");
                snprintf(lines[3], MAX_LEN, "vel=0.0,acc=5:ir2>0.4");
                snprintf(lines[4], MAX_LEN, "vel=0.0,acc=5:time=0.1");
				snprintf(lines[5], MAX_LEN, "vel=0.5,acc=5, edgel=0.0, white=1:xl>16,ir2<0.25");
				snprintf(lines[6], MAX_LEN, "vel=0.0,acc=100.0:time=0.1");
				snprintf(lines[7], MAX_LEN, "tr=0.0,vel=0.3,acc=3:turn=90");
				snprintf(lines[8], MAX_LEN, "vel=0.4,acc=2,edgel=0.0, white=1 :dist=3");
				snprintf(lines[9], MAX_LEN, "event=1:time=1.1");
                missionSendAndRun(lineList, 10);
                state++;
                break;
        case 201:// goal 
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






// kun som backup bruges til at kopiere 
bool UMission::setRacing(int & state)
{
bool finished = false;

switch (state)
{
    case 0: // first PART 
    bot->send("cedg 1 0.02 0 1 1e+06 1 1 0.4 0.1 0 1 1 0 1 1 0 1 1e+06 1 0 1 0 1 1 0 1e+06\n\0"); //set racing line-parameters
    snprintf(lines[0], MAX_LEN, "event=1:time=1.1");
    missionSendAndRun(lineList, 1);
    state++;
    
    break;

        if (bot->event->eventSet(1))
    { // finished first drive
        state = 999;
//         printf("mission finished first part\n");
    }

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
    bot->send("cedg 1 0.08 0 1 1e+06 1 1 0.4 0.1 0 1 1 0 1 1 0 1 1e+06 1 0 1 0 1 1 0 1e+06\n\0"); //set normal line-parameters
    snprintf(lines[0], MAX_LEN, "event=1:time=1.1");
    missionSendAndRun(lineList, 1);

    state++;
    break;

        if (bot->event->eventSet(1))
    { // finished first drive
        state = 999;
//         printf("mission finished first part\n");
    }
    case 999:
    default:
    finished = true;
    break;
}
return finished;
}

