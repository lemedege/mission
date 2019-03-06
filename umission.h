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
  UBridge * bot;
  UCamera * cam;
  // read thread handle
  thread * th1 = NULL;
  // set true to stop thread
  bool th1stop = false;
  bool active = false;
  // thread on Regbot
  int threadActive = false;
  /// space for fabricated lines
  const static int MAX_LINES = 5;
  const static int MAX_LEN = 100;
  char lines[MAX_LINES][MAX_LEN];
  // make an array of pointers to mission lines
  const char * lineList[MAX_LINES];
  
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
  /** which missions to run */
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
  bool mission2(int & state)
  { // should be replaced with something better
    return mission1(state);
  }
  

  bool mission3(int & state);

  
private:
  void missionSendAndRun(const char * missionLines[], int missionLineCnt);
  static const int missionLineMax = 10;
};


#endif
