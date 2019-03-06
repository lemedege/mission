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

#ifndef UREGBOT_H
#define UREGBOT_H

// #include <iostream>
#include <sys/time.h>
#include <cstdlib>
// #include <sstream>
#include <thread>
#include <mutex>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <math.h>
// #include <opencv2/core/core.hpp>
// #include <opencv2/highgui/highgui.hpp>
#include "urun.h"
#include "tcpCase.h"
#include "utime.h"

using namespace std;
// forward declaration
class UBridge;

/////////////////////////////////////////////////////////////

/**
 * base class for data items from robot and bridge */
class UData
{ // base class for data
public:
  timeval dataTime;
  UBridge * bridge = NULL;
  // set update time
  void updated()
  {
    gettimeofday(&dataTime, NULL);
  }
  // get time since update
  float getTimeSinceUpdate()
  {
    timeval t;
    gettimeofday(&t, NULL);
    return getTimeDiff(t, dataTime);
  }
  // just a default reply
  // implement in real class when needed
  virtual bool tick()
  {
    return false;
  }
  // subscribe to data
  virtual void subscribe() {};
};

/**
 * robot pose info */
class UPose : public UData
{
public:
  float x = 0.0;
  float y = 0.0;
  float h = 0.0;
  float dist = 0.0;
  FILE * poselog;
  // constructor
  UPose(UBridge * bridge_ptr);
  // destructor 
  ~UPose();
  //
  void decode(char * msg);
  //
  virtual void subscribe();
};

/////////////////////////////////////////////////////////////

/**
 * Line sensor data */
class UEdge  : public UData
{
public:
  // edge detect
  bool edgeValidLeft = false;
  bool edgeValidRight = false;
  bool edgeCrossingBlack = false;
  bool edgeCrossingWhite = false;
  //
  UEdge(UBridge * bridge_ptr);
  void decode(char * msg);
  void subscribe() override;
  
};

/////////////////////////////////////////////////////////////

/**
 * Other info like mission line and battery voltage */
class UInfo  : public UData
{
public:
  float regbotTime = 0.0;
  timeval bootTime;
  float batteryVoltage = 0.0;
  bool missionRunning = false;
  int missionLineNum = 0;
  int missionThread = 0;
  // methods
  UInfo(UBridge * bridge_ptr);
  void decodeHbt(char * msg);
  void decodeMission(char * msg);
  float getTime();
  
  void subscribe() override;
  
};

/////////////////////////////////////////////////////////////

class UEvent  : public UData
{
public:
  static const int MAX_EVENT_FLAGS = 34;
  bool eventFlags[MAX_EVENT_FLAGS];
  mutex eventUpdate;
  
  UEvent(UBridge * bridge_ptr);
  //
  void decode(char * msg);
  /** set event flag */
  void setEvent(int eventNumber);
  /**
   * Clear all event flags to false */
  void clearEvents();
  /**
   * Requests if this event has occured.
   * \param event the event flag to test
   * \returns true and resets event, if it was set */
  bool eventSet ( int event );
  
  void subscribe() override;
};

/////////////////////////////////////////////////////////////

class UJoy  : public UData
{
public:
  int axis[8] = {0,0,0,0,0,0,0};
  // axis 0 is left hand left-right axis
  // axis 1 is left hand up-down axis (servo)
  // axis 2 is left front speeder
  // axis 3 is right hand left-right (turn)
  // axis 4 is right hand up-down (velocity)
  // axis 5 is right front speeder
  // axis 6 id digital left-write
  // axis 7 is digital up-down 
  bool button[11] = {0,0,0,0,0,0,0,0,0,0,0};
  // first 4 is [0]=green, [1]=red, [2]=blue, [3]=yellow
  bool manual = false;
  
  UJoy(UBridge * bridge_ptr);
  //
  void decode(char * msg);
  
  void subscribe() override;
};

/////////////////////////////////////////////////////////////

class UMotor  : public UData
{
public:
  float velocity[2];
  float current[2];
  
  UMotor(UBridge * bridge_ptr);
  //
  void decodeVel(char * msg);
  void decodeCurrent(char * msg);
  //
  void subscribe() override;
};

/////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////
/**
 * The robot class handles the 
 * port to the REGBOT part of the robot through the bridge,
 * REGBOT handles the most real time issues and the bridge a few
 * and this class is the interface to that. */
class UBridge : public URun, public tcpCase
{ // REGBOT interface
public:
  // data items
  UPose * pose = new UPose(this);
  UEdge * edge = new UEdge(this);
  UInfo * info = new UInfo(this);
  UEvent * event = new UEvent(this);
  UJoy * joy = new UJoy(this);
  UMotor * motor = new UMotor(this);
  // debug log
  FILE * botlog;
  
private:
  // read thread handle
  thread * th1;
  // set true to stop thread
  bool th1stop;
  // mutex to ensure commands to regbot is not mixed
  mutex sendMtx;
  mutex logMtx;
  // receive buffer
  static const int MAX_RX_CNT = 500;
  char rx[MAX_RX_CNT];
  // number of characters in rx buffer
  int rxCnt;
  // status message sequence (fast (sensors))
  int tickClassIdx;
  // status message sequence (slow ("static" info))
  int statusMsgIdx2;
  
public:
  /** constructor
   * \param server is a string with either IP address or hostname.
   * connects to port 24001
   */
  UBridge(const char * server);
  /** destructor */
  ~UBridge();
  /**
   * send a string to the serial port */
  void send(const char * cmd);
  /**
   * receive thread */
  void run();
  /**
   * clear events */
  void clearEvents();
  /**
   * test and reset event */
  bool eventSet(int event);
  /**
   * Set an event flag
   * \param eventNumber is event to set (in range 0..33) */
  void setEvent(int eventNumber);
  /**
   * stop interface */
  void stop();
  /**
   * Get time since boot in float seconds */
//   float getTime()
//   {
//     timeval t;
//     gettimeofday(&t, NULL);
//     return getTimeDiff(t, bootTime);
//   }
  
private:
  /**
   * Open the connection.
   * \returns true if successful */
//   bool openToRegbot();
  /**
   * decode messages from REGBOT */
  void decode(char * msg);
  /** decode event message */
//   void decodeEvent(char * msg);
  /** decode heartbeat message */
  void decodeHbt(char * msg);
  /** decode heartbeat message */
  void decodePose(char * msg);
  /** decode line sensor edge message */
  void decodeEdge(char * msg);
  /** decode mission status - running or not */
  void decodeMissionStatus(char * msg);
  /** send regulat status requests, if no other traffic */
  void requestDataTick();
  /**
   * time (for debug print) */
  UTime t;
};

#endif
