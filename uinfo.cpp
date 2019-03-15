/***************************************************************************
 *   Copyright (C) 2019 by DTU (Christian Andersen)                        *
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

#include <string.h>
#include "ubridge.h"

UInfo::UInfo(UBridge * bridge_ptr, bool openLog)
{
  bridge = bridge_ptr;
  gettimeofday(&bootTime, NULL);
  // default
  strncpy(robotname, "no name", MAX_NAME_LENGTH);
}


void UInfo::decodeHbt(char * msg)
{ // "hbt 2399.4 12.1"
  char * p1 = &msg[3];
  regbotTime = strtof(p1, &p1);
  batteryVoltage = strtof(p1, &p1);
}

void UInfo::decodeId(char * msg)
{ // rid 84 0.206 9.68 48 0.04 0.04 0 1 9.89802 6 Cleo
  // snprintf(reply, MRL, "rid %d %g %g %d %g %g %g %d %g %d %s\r\n",
  //   robotId, 
  //   odoWheelBase, gear, 
  //   pulsPerRev,
  //   odoWheelRadius[0], odoWheelRadius[1],
  //   balanceOffset,
  //   batteryUse, batteryIdleVoltageInt * batVoltIntToFloat,
  //   robotHWversion,
  //   robotname[robotId]
  //   ); 
  char * p1 = &msg[3];
  robotId = strtol(p1, &p1, 10);
  odoWheelBase = strtof(p1, &p1);
  gear = strtof(p1, &p1);
  pulsPerRev = strtol(p1, &p1, 10);
  odoWheelRadius[0] = strtof(p1, &p1);
  odoWheelRadius[1] = strtof(p1, &p1);
  balanceOffset = strtof(p1, &p1);
  batteryUse = strtol(p1, &p1, 10);
  batteryIdleVoltage = strtof(p1, &p1);
  robotHWversion = strtol(p1, &p1, 10);
  strncpy(robotname, p1, MAX_NAME_LENGTH);
}

void UInfo::decodeMission(char * msg)
{
  /* snprintf(s, MSL, "mis %d %d %d '%s' %d %d\r\n",
   *         mission, missionState , missionLineNum, 
   *         missionName[mission],
   *         0, // unused (was sendStatusWhileRunning)
   *         misThread
   * */
  //     printf("Uinfo::decodeMission - got %s\n", msg); 
  char * p1 = &msg[3];
  /*int midx =*/ strtol(p1, &p1, 0);
  int misState = strtol(p1, &p1, 0);
  missionRunning = misState == 2;
  missionLineNum = strtol(p1, &p1, 0);
  while (*p1 > ' ' and *p1 != '\'')
    p1++;
  p1 = strchr(++p1, '\'');
  if (p1 != NULL)
  {
    p1++;
    int n = strtol(p1, &p1, 0);
    if (n != 0)
      printf("UInfo::decodeMission error in format (5) (%s)\n", msg);
    missionThread = strtol(p1, &p1, 0);
    // when mission is started, mission state is 1 for 1.0 seconds, then 2 when mission is running
  }
  else
    printf("UInfo::decodeMission error in format (4) (%s)\n", msg);
  // timestamp data
  gettimeofday(&dataTime, NULL);
}

float UInfo::getTime()
{
  timeval t;
  gettimeofday(&t, NULL);
  return getTimeDiff(t, bootTime);
}

void UInfo::subscribe()
{
  bridge->send("mis subscribe 2\n"); // mission data
  bridge->send("hbt subscribe 1\n"); // time and mission info  
  bridge->send("rid subscribe 3\n"); // get robot ID values
  bridge->send("robot u4\n"); // request ID values from robot (once should be enough)
}
