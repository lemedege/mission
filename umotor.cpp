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



UMotor::UMotor(UBridge * bridge_ptr, bool openLog)
{
  bridge = bridge_ptr;
}
//


void UMotor::decodeVel(char * msg)
{ //   snprintf(reply, MRL,"wve %g %g\r\n", wheelVelocityEst[0], wheelVelocityEst[1]);
  char * p1 = &msg[4];
  velocity[0] = strtof(p1, &p1);
  velocity[1] = strtof(p1, &p1);
}


void UMotor::decodeCurrent(char * msg)
{ //   snprintf(reply, MRL,"wve %g %g\r\n", wheelVelocityEst[0], wheelVelocityEst[1]);
  char * p1 = &msg[4];
  current[0] = strtof(p1, &p1);
  current[1] = strtof(p1, &p1);
}


void UMotor::subscribe()
{
  bridge->send("wve subscribe 2\n"); // velocity
  bridge->send("mca subscribe 2\n"); // motor current  
}

