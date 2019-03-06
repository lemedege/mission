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

UEvent::UEvent(UBridge * bridge_ptr)
{
  clearEvents();
  bridge = bridge_ptr;
}
//
void UEvent::decode(char * msg)
{ // "event 33"
  // skip the first 5 characters
  char * p1 = &msg[5];
  int eventNumber = strtol(p1, &p1,0);
  setEvent(eventNumber);
  // debug
  printf("# got event %d: %s\n", eventNumber, msg);
}
/** set event flag */
void UEvent::setEvent(int eventNumber)
{
  if (eventNumber < MAX_EVENT_FLAGS and eventNumber >= 0)
  { // event 33 is start button, event 0 is misson stop
    eventUpdate.lock();
    printf("# Event received: %d\n", eventNumber);
    eventFlags[eventNumber] = true;
    eventUpdate.unlock();
  }
}
/**
 * Clear all event flags to false */
void UEvent::clearEvents()
{
  for (int i = 0; i < MAX_EVENT_FLAGS; i++)
    eventFlags[i] = false;
}
/**
 * Requests if this event has occured.
 * \param event the event flag to test
 * \returns true and resets event, if it was set */
bool UEvent::eventSet ( int event )
{
  bool set = false;
  if (event <= MAX_EVENT_FLAGS and event >= 0)
  { // make sure it is not updated between test and set
    //       bool events = false;
    //       for (int i = 0; i < MAX_EVENT_FLAGS; i++)
    //       {
    //         if (eventFlags[i] != 0)
    //         {
    //           events=true;
    //           if (event > 0)
    //             printf("UEvent::eventSet:: - looking for %d, event=%d is set\n", event, i);
    //         }
    //       }
    //       if (events)
    { // test and clear event
      eventUpdate.lock();
      set = eventFlags[event];
      eventFlags[event] = 0;
      eventUpdate.unlock();
    }
  }
  return set;
}

void UEvent::subscribe()
{
  clearEvents();
  bridge->send("event subscribe 6\n"); // start, stop and mission events
  bridge->send("event get\n"); // start, stop and mission events
}

