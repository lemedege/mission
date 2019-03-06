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

#include "ubridge.h"


UPose::UPose(UBridge * bridge_ptr)
{
  bridge = bridge_ptr;
  if (false)
  { // open pose log
    const int MNL = 100;
    char date[MNL];
    char name[MNL];
    UTime time;
    time.now();
    time.getForFilename(date);
    snprintf(name, MNL, "log_odometry_%s.txt", date);
    poselog = fopen(name, "w");
    if (poselog != NULL)
    {
      fprintf(poselog, "%% robobot poselog\n");
      fprintf(poselog, "%% 1 Timestamp in seconds\n");
      fprintf(poselog, "%% 2 x (forward)\n");
      fprintf(poselog, "%% 3 y (left)\n");
      fprintf(poselog, "%% 4 h (heading in radians)\n");
    }
  }
  else
    poselog = NULL;
}


UPose::~UPose()
{
  if (poselog != NULL)
    fclose(poselog);
}
//

void UPose::decode(char * msg)
{ // assuming msg = "pse ..."
  char * p1 = &msg[3];
  float x2 = x, y2 = y;
  x = strtof(p1, &p1);
  y = strtof(p1, &p1);
  h = strtof(p1, &p1);
  dist += hypot(x - x2, y - y2);
  if (poselog != NULL)
  {
    UTime t;
    t.now();
    fprintf(poselog, "%ld.%03ld %.3f %.3f %.4f\n", t.getSec(), t.getMilisec(), x, y, h);
  }
}

void UPose::subscribe()
{
  bridge->send("pse subscribe 1\n"); // Pose
}
