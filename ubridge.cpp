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

// #include <iostream>
#include <sys/time.h>
#include <cstdlib>
// #include <fstream>
// #include <sstream>
#include <thread>
#include <mutex>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <math.h>
#include <string.h>
#include <termios.h>
// #include <opencv2/core/core.hpp>
// #include <opencv2/highgui/highgui.hpp>

#include "ubridge.h"

using namespace std;


//////////////////////////////////////////////////////////////////

/** constructor */
UBridge::UBridge(const char * server)
{
  timeval t;
  th1stop = false;
  th1 = NULL;
  tickClassIdx = 0;
  printf("UBridge:: opening socket to bridge\n");
//  createSocket("24001", "localhost");
//  createSocket("24001", "hannah.local");
//  createSocket("24001", "127.0.0.1");
  createSocket("24001", server);
  tryConnect();
  if (connected)
  {
    printf("UBridge:: connected to bridge\n");
    th1 = new thread(runObj, this);
  }
  botlog = NULL;
  if (false)
  {
    const int MNL = 100;
    char date[MNL];
    char name[MNL];
    UTime time;
    time.now();
    time.getForFilename(date);
    snprintf(name, MNL, "log_rx_tx_%s.txt", date);
    botlog=fopen(name,"w");
  }
  if (botlog != NULL)
  {
    gettimeofday(&t, NULL);
    float dt = getTimeDiff(t, info->bootTime);
    logMtx.lock();
    fprintf(botlog, "%6.3f->: %s %d\n", dt, "Connected to bridge ", connected);
    fflush(botlog);
    logMtx.unlock();
  }
}
/** destructor */
UBridge::~UBridge()
{ // stop all activity before close
  stop();
  if (botlog != NULL)
    fclose(botlog);
}

void UBridge::stop()
{
  th1stop = true;
  if (th1 != NULL)
    th1->join();
  printf("Regbot:: closed usb port\n");
}
/**
  * send a string to the serial port */
void UBridge::send(const char * cmd)
{ // this function may be called by more than one thread
  // so make sure that only one send at any one time
//   printf("UBridge::send 0\n");
  sendMtx.lock();
  if (connected)
  { // send data
//     sleep(1);
//    printf("UBridge::send 2-\n");
    sendData(cmd);
//     printf("UBridge::send 2+ %s", cmd);
    // debug
    {
      timeval t;
      gettimeofday(&t, NULL);
      float dt = getTimeDiff(t, info->bootTime);
      if (botlog != NULL)
      {
        logMtx.lock();
        fprintf(botlog, "%6.3f->: %s", dt, cmd);
        fflush(botlog);
        logMtx.unlock();
      }
      printf("UBridge::send %6.3f->: %s", dt, cmd);        
    }
  }
  // 
  usleep(4000); // Short sleep helps communication be more reliable when many command lines are send consecutively.
  sendMtx.unlock();
}
/**
  * receive thread */
void UBridge::run()
{ // read thread for REGBOT messages
  int n = 0;
  rxCnt = 0;
  timeval idleTime;
  gettimeofday(&idleTime, NULL);
  bool sockErr = false;
  // get robot name
  //send("u4\n");
  int loop = 0;
  while (not th1stop)
  {
    char c;
    n = readChar(&c, &sockErr);
    if (n == 1)
    { // not an error or hangup
      if (c >=' ' or c == '\n')
      { // got a valid character
        rx[rxCnt] = c;
        if (rx[rxCnt] == '\n')
        { // terminate string
          rx[rxCnt] = '\0';
//           printf("#### Received: (loop=%d, rxCnt=%d) %s",loop, rxCnt, rx);
          decode(rx);
          rxCnt = 0;
          memset(rx, '\0', MAX_RX_CNT);
          n = 0;
          gettimeofday(&idleTime, NULL);
          continue;
        }
        // prepare for next byte
        if (rxCnt < MAX_RX_CNT)
          rxCnt++;
        else
        {
          printf("UBridge::run: receiver overflow\n");
          rxCnt = 0;
        }
      }
    }
    else if (not sockErr)
    {
      timeval t2;
      gettimeofday(&t2, NULL);
      float dt = getTimeDiff(t2, idleTime);
      if (dt > 0.005)
      { // waited more than 10 ms - do something
        requestDataTick();
        idleTime = t2; // not idle anymore
      }
      usleep(1000);
    }
    else
    {
      perror("Regbot:: port error");
      usleep(100000);
    }
    usleep(200);
    loop++;
  }
  printf("Regbot:: thread stopped\n");
}  


/**
  * decode messages from REGBOT */
void UBridge::decode(char * message)
{
  // skip whitespace
  while (*message <= ' ' and *message > '\0')
    message++;
  // debug
  // printf("UBridge::decode:: got:%s\n", message);
  // debug end
  if (*message != '\0')
  {
    if (strncmp(message, "hbt ", 4) == 0)
      info->decodeHbt(message); // it is a heartbeat message (time and battry voltage)
    else if (strncmp(message, "pse ", 4) == 0)
      pose->decode(message); // it is a pose message
    else if (strncmp(message, "lip ", 4) == 0)
      edge->decode(message); // it is a line edge message
    else if (strncmp(message, "event", 5)==0)
    {
      event->decode(message);
      t.now();
      printf("%ld.%03ld UBridge::decode: %s\n", t.getSec(), t.getMilisec(), message);
    }
    else if (strncmp(message, "mis ", 4)==0)
      info->decodeMission(message); // mission status skipped
    else if (strncmp(message, "joy ", 4)==0)
    {
      joy->decode(message); // 
    }
    else if (strncmp(message, "wve ", 4)==0)
    { // wheel velocity
      motor->decodeVel(message);
    }
    else if (strncmp(message, "mca ", 4)==0)
    { // motor current
      motor->decodeCurrent(message);
    }
    else if (strncmp(message, "rid ", 4)==0)
      ; // robot ID skipped
    else if (*message == '#')
      // just a message from Regbot
      printf("%s", message);
    else
    { // not yet supported message - ignore
      printf("Regbot:: unhandled message: '%s'\n", message);
    }
  }
  if (botlog != NULL)
  {
    timeval t;
    gettimeofday(&t, NULL);
    float dt = getTimeDiff(t, info->bootTime);
    logMtx.lock();
    fprintf(botlog, "%6.3f<-: %s\n", dt, message);
    logMtx.unlock();
  }
}

//////////////////////////////////////////////////

/** send regulat status requests, if no other traffic */
void UBridge::requestDataTick()
{
  int cnt = 0;
  bool used = false;
  while (cnt <= 5 and not used)
  {
    switch (tickClassIdx)
    {
      /*
       *  UPose * pose = new UPose(this);
       *  UEdge * edge = new UEdge(this);
       *  UInfo * info = new UInfo(this);
       *  UEvent * event = new UEvent(this);
       *  UJoy * joy = new UJoy(this);
       *  UMotor * motor = new UMotor(this);
       * */
      case 0: used = pose->tick();  break;
      case 1: used = edge->tick();  break;
      case 2: used = info->tick();  break;
      case 3: used = event->tick();  break;
      case 4: used = joy->tick();  break;
      case 5: used = motor->tick(); break;
      default:
        tickClassIdx = 0;
        break;
    }
    tickClassIdx++;
    cnt++;
  }
}


