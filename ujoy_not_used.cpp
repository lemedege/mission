/***************************************************************************
 *   Copyright (C) 2017 by DTU (Christian Andersen)                        *
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

#include <sys/ioctl.h>
#include <signal.h>
#include <linux/joystick.h>

#include "ujoy.h"

using namespace std;

// start is green button
#define BUTTON_START   0
// red button is 1
// blue button is 2
// image is yellow button (3)
#define BUTTON_IMAGE   3
// left front button is 4
// right front button is fast
#define BUTTON_FAST    5
// goto manuel mode is all 3 upper-center buttons
#define BUTTON_MANUAL1 6
#define BUTTON_MANUAL2 7
#define BUTTON_MANUAL3 8
// axis 0 is left hand left-right axis
// axis 1 is left hand up-down axis
#define AXIS_SERVO_2   1
// axis 2 is left front speeder
// axis 3 is right hand left-right
#define AXIS_TURN      3
// axis 4 is right hand up-down
#define AXIS_VEL       4
// axis 5 is right front speeder
// axis 6 id digital left-write
// axis 7 is digital up-down
// parameters for JOY control (velocity is in m/s)
#define VELOCITY_MAX          2.5
#define VELOCITY_TURN_MAX     0.5
#define VELOCITY_SERVO_MAX   30.0
// factor for normal speed
#define VELOCITY_SLOW_FACTOR  0.33

void UJoy::stop()
{
  th1stop = true;
//   if (joyRunning)
//     th1->join();
  printf("Joy:: stopped\n");
}

/////////////////////////////////////////////////////////////////////////////////

/** read form joystick driver */
bool UJoy::getNewJsData()
{
  struct js_event jse;
  
  int bytes = read(jDev, &jse, sizeof(jse));
  
  if (bytes == -1) 
  {
    printf(" JOY control: Failed reading from joystick - Exiting!\n");
    return false;
  } 
  else if (bytes != sizeof(jse)) 
  { // size error
    printf("JOY control : Unexpected bytes from joystick:%d - continues\n", bytes);
  } 
  else 
  { //Proper joystick package has been recieved
    //Joystick package parser
    jse.type &= ~JS_EVENT_INIT; /* ignore synthetic events */
    switch(jse.type) {
      // changed axis position
      case JS_EVENT_AXIS:
        if (jse.number < 16) {
          joyValues.axes[jse.number] = jse.value;
        }
        break;
      // changed button state
      case JS_EVENT_BUTTON:
        if (jse.number < 16) {
          joyValues.button[jse.number] = jse.value;
        }
        break;
      default:
        break;
    }
  }
  return true;
}

//////////////////////////////////////////////////////////

void UJoy::initManualControl()
{
  printf("JOY:: in manuel control\n");
  bot->send("rc=1 0 0\n");  
//   usleep(10000);
//   // enable servos for steering and big servo
//   bot->send("svo 1 0 0  1 0 0  0 0 0  0 0 0  0 0 0\n");
//   usleep(10000);
//   // enable servo 1 as steering
//   bot->send("sv1 1 0 0.16 90.0\n");
}

//////////////////////////////////////////////////////////

void UJoy::runManualControl()
{
  const int MSL = 50;
  char s[MSL];
  // test servo position
  if (fabs(joyValues.axes[AXIS_SERVO_2]) > 500)
  { // move servo
    servo2Pos += float(joyValues.axes[AXIS_SERVO_2]) / 32000.0 * VELOCITY_SERVO_MAX * fastScale;
    if (servo2Pos > 1000)
      servo2Pos = 1000;
    else if (servo2Pos < -1000)
      servo2Pos = -1000;
    // debug
    // printf("Joy:: %.3f servo2 %d (pos = %g)\n", bot->getTime(), joyValues.axes[AXIS_SERVO_2], servo2Pos);
    // debug end
  }
  if (fabs(servo2Pos - servo2Pos_old) > 2.0)
  {
    snprintf(s, MSL, "servo 2 %d\n", int(round(servo2Pos)));
    bot->send(s);
    servo2Pos_old = servo2Pos;
//     printf("Joy:: %.3f RC servo control send: %s", bot->getTime(), s);
  }
  else
  { // send velocity and turn
    snprintf(s, MSL, "rc=1 %g %g\n", velocity, turnVelocity);
    bot->send(s);
    //   printf("Mission:: RC control send: %s", s);
  }
}

//////////////////////////////////////////////////////////

void UJoy::stopManualControl()
{
  bot->send("rc=0 0 0\n");  
  printf("JOY:: return to auto mode\n");
}

/////////////////////////////////////////////////////////////////////////////////

bool UJoy::testJoy(bool inManualControl)
{
  if (manOverride)
  { // do not run automatic mission
    if (not inManualControl)
    { // just entered manual control 
      initManualControl();
      inManualControl = true;
    }
    else
      runManualControl();
  }
  else if (inManualControl)
  {
    stopManualControl();
    inManualControl = false;
  }
  return inManualControl;
}

/////////////////////////////////////////////////////////////////////////////////

void UJoy::joyControl()
{ // called when js driver sees a change (may be slow)
  if (joyValues.button[BUTTON_FAST])
    fastScale = 1.0;
  else
    fastScale = VELOCITY_SLOW_FACTOR;
  // velocity
  if (fabs(joyValues.axes[AXIS_VEL]) > 500)
  { // velocity is valid
    velocity = -float(joyValues.axes[AXIS_VEL]) / 32000.0 * fastScale * VELOCITY_MAX;
  }
  else
    velocity = 0.0;
  // turn
  if (fabs(joyValues.axes[AXIS_TURN]) > 500)
  { // velocity is valid
    turnVelocity = -float(joyValues.axes[AXIS_TURN]) / 32000.0 * VELOCITY_TURN_MAX;
  }
  else
    turnVelocity = 0.0;
  // start
  if (joyValues.button[BUTTON_START])
  { // fake that start is pressed
    bot->event.setEvent(33);
    // exit remote control state
    enterAutoState();
  }
  if (joyValues.button[BUTTON_START])
  { // fake that start is pressed
    bot->event.setEvent(33);
    // exit remote control state
    enterAutoState();
  }
  if (joyValues.button[BUTTON_IMAGE])
  {
    
  }
}

/////////////////////////////////////////////////////////////////////////////////

void UJoy::enterAutoState()
{
  manOverride = false;
  fprintf(stdout, "Joy::Enter auto mode\n");
}

/////////////////////////////////////////////////////////////////////////////////

void UJoy::enterManualState()
{
  manOverride = true;
  fprintf(stdout, "Joy::Enter manual mode\n");
}

/////////////////////////////////////////////////////////////////////////////////

void UJoy::run()
{
   joyRunning = initJoy();  
   if (not joyRunning)
   { // no such device
     fprintf(stderr, "Can't open joystick port: %s\n",joyDevice);
   }
   while (not th1stop)
   {
     if (not joyRunning)
     {
       sleep(1);
       // retry
       joyRunning = initJoy();  
     }
     else
     { // Device is present
       joyRunning = getNewJsData();
       //Detect manual override toggling
       if (((joyValues.button[BUTTON_MANUAL1] == 1) or 
            (joyValues.button[BUTTON_MANUAL2] == 1) or
            (joyValues.button[BUTTON_MANUAL3] == 1)
           ) and not toggleState)
       {
         if (manOverride) 
           enterAutoState();
         else
           enterManualState();
         //Set state for toggleing - wait for button release
         toggleState = 1; 
       } 
       else if (!((joyValues.button[BUTTON_MANUAL1] == 1) or 
                  (joyValues.button[BUTTON_MANUAL2] == 1) or
                  (joyValues.button[BUTTON_MANUAL3] == 1)))
       {
         toggleState = 0;
       }
       if (manOverride)
         // we are in manual mode, so
         joyControl();
     }
   }
   if (joyRunning) 
   { // close device nicely
     joyRunning = false;
     if (jDev) 
       close(jDev);
   }
}


bool UJoy::initJoy(void) 
{ //Open first serial js port
  jDev = open (joyDevice, O_RDWR /*| O_NONBLOCK*/);
  if (jDev >= 0)
  { // Joystic device found
    printf("Opened joystick on port: %s\n",joyDevice);
    //Query and print the joystick name
    char name[128];
    if (ioctl(jDev, JSIOCGNAME(sizeof(name)), name) < 0)
      strncpy(name, "Unknown", sizeof(name));
    printf("Joystick model: %s\n", name);    
    //Query and print number of axes and buttons
    ioctl (jDev, JSIOCGAXES, &number_of_axes);
    ioctl (jDev, JSIOCGBUTTONS, &number_of_buttons);
    printf("Registrered %d axes and %d buttons on joystick\n",number_of_axes,number_of_buttons);
  }
  return jDev >= 0;
}  
  
  