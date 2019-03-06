
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

#include <iostream>
#include <sys/time.h>
#include <cstdlib>
#include <fstream>
#include <sstream>
#include <thread>
#include <mutex>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <signal.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

//#include "../Downloads/raspicam-0.0.5/src/raspicam.h"

// #define PI_CAM
#ifdef PI_CAM
#include <raspicam/raspicam.h>
#endif
#include "urun.h"
#include "ucamera.h"
#include "ubridge.h"
#include "umission.h"
// #include "ujoy.h"

using namespace std;



void printHelp(char * name)
{ // show help
  printf("\nUsage: %s [<from mission> [<to mission>]]\n\n", name);
  printf("<from mission> and <to mission>:\n");
  printf("     number in the range 1..998, and the code\n");
  printf("     run only the mission parts in this range.\n\n");
  printf("E.g.: './%s 2 2' runs mission 2 only\n\n", name);
  printf("NB!  Robot may continue if this app is stopped with ctrl-C.\n\n");
}

////////////////////////////////////////////////////////////////////

int main ( int argc,char **argv ) 
{
  int firstMission = 1, lastMission = 998;
  // are there mission parameters
  if (argv[1] != NULL and (*argv[1] == '-' or *argv[1] == 'h'))
  {
    printHelp(argv[0]);
  }
  else 
  { // not help - create modules
    if (argc > 1)
    { // get mission range
      firstMission = strtol(argv[1], NULL, 10);
      lastMission = firstMission;
      if (argc > 2)
        lastMission = strtol(argv[2], NULL, 10);
    }
    // create connection to robot
    UBridge reg("127.0.0.1"); // localhost, when running in raspberry
//    UBridge reg("10.16.160.175"); // thit
    // create camera interface
    UCamera cam(&reg);
    // create mission 
    UMission mission(&reg, &cam);
    // get mission range (default is 1..988 (all))
    mission.fromMission = firstMission;
    mission.toMission = lastMission;
    mission.start();
    // start requested missions
    printf("Press h for help (q for quit)\n");
    cam.start();
    //
    usleep(100000);
    printf(">> ");
    fflush(NULL);
    int flags = fcntl(stdin->_fileno, F_GETFL, 0); 
    fcntl(stdin->_fileno, F_SETFL, flags | O_NONBLOCK);
    //
    while (reg.connected and not mission.finished)
    { // Just wait for quit
      char c = ' ';
      int n = read(stdin->_fileno, &c, 1);
      if (n > 0)
      {
        if (c == 'q' and n == 1)
          mission.stop();
        else if (c == 'c')
        {
          printf("# setting flag to capture and save image\n");
          cam.saveImage = true;
        }
        else if (c == 's')
        {
  //         printf("# setting start flag (event 33)\n");
          reg.event->setEvent(33);
        }
        else if (c == 'h')
        {
          printf("# mission command options\n");
          printf("#    h    This help\n");
          printf("#    c    Capture an image and save to disk (image*.png)\n");
          printf("#    s    Start mission\n");
        }
        usleep(10000);
        if (n == 1)
        {
          printf(">> ");
          fflush(NULL);
        }
      }
      usleep(300);
      //mission.runMission(firstMission, lastMission);
    }
    printf("Mission ended.\n");
  }
}
