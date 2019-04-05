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

#ifndef UMISSION_H
#define UMISSION_H


#include <sys/time.h>
#include <cstdlib>
#include "urun.h"
#include "ucamera.h"
#include "ubridge.h"
// #include "ujoy.h"


/**
 * Base class, that makes it easier to starta thread
 * from the method runObj
 * The run method should be overwritten in the real class */
class UMission : public URun
{
public:
  /// flag to finish all (pending) missions and RC
  bool finished = false;
private:
  /**
   * Pointer to communication and data part of this mission application */
  UBridge * bot;
  /**
   * Pointer to camera (class) of this mission application */
  UCamera * cam;
  /** is thread active */
  bool active = false;
  /** is the number of the active thread in the REGBOT */
  int threadActive;
  /** space for fabricated lines
   * maximum number of lines */
  /**
   * maximum line count for the biggest mission snippet 
   * (limit is probably around 20 with version 4.1 (red) base board) */
  static const int missionLineMax = 15;
  /** maximum length of one mission line (in characters) */
  const static int MAX_LEN = 100;
  /** definition of array with c-strings for the mission snippets */
  char lines[missionLineMax][MAX_LEN];
  /** an array of pointers to mission lines */
  const char * lineList[missionLineMax];
  
public:
  /**
   * Constructor */
  UMission(UBridge * regbot, UCamera * camera);
  /**
   * Destructor */
  ~UMission();
  /**
   * Initialize regbot part to accept mission commands */
  void missionInit();  
  /**
   * Run the missions
   * \param fromMission is first mission element (default is 1)
   * \param toMission is last mission to run (default is 998)
   * so 'runMission(3,3)' runs just mission part 3 */
  void runMission();
  /**
   * Thread run function */
  void run();
  /**
   * Activate mission */
  void start()
  {
    active = true;
    printf("UMission::start active=true\n");
  }
  /**
   * Stop all missions */
  void stop()
  {
    finished = true;
    th1stop = true;
    usleep(11000);
    if (th1 != NULL)
      th1->join();
  }
  /** which missions to run 
   * These values can be set as parameters, when starting the mission */
  int fromMission;
  int toMission;
private:
  /**
   * Mission part 1 
   * \return true, when finished */
  bool mission1(int & state);
  /**
   * Mission part 2
   * \return true, when finished */
  bool mission2(int & state);
  

  bool mission3(int & state);


  bool racetrack(int & state);
  bool racetrack2(int & state);
  bool axeGate(int & state);
  
  bool setRacing(int & state);

  bool setNormal(int & state);

private:
  /**
   * Send a number of lines to the REGBOT in a dormant thread, and 
   * make these lines active - stopping the last set of lines.
   * \param missionLines is a pointer to an array of c-strings
   * \param missionLineCnt is the number of strings to be send from the missionLine array. */
  void missionSendAndRun(const char * missionLines[], int missionLineCnt);
};


#endif
