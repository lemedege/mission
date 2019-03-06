/*
 *Joystick control */

#ifndef UJOY_H
#define UJOY_H


#include <sys/time.h>
#include <cstdlib>
#include "urun.h"
#include "ucamera.h"
#include "ubridge.h"

#define JS_EVENT_BUTTON         0x01    /* button pressed/released */
#define JS_EVENT_AXIS           0x02    /* joystick moved */
#define JS_EVENT_INIT           0x80    /* initial state of device */

class UJoy : public URun
{
  
public:
  /** constructor */
  UJoy(UBridge * regbot, UCamera * picam)
  { /** robot interface */
    bot = regbot;
    cam = picam;
    /** thread to receive new joy events */
    th1 = NULL;
    th1stop = false;
    th1 = new thread(runObj, this);
    /** servo position in range -1000 to 1000 */
    servo2Pos = 0.0;
    servo2Pos_old = 0.0;
  }
  /** destructor */
  ~UJoy()
  {
    stop();
  }
  /**
   * Run joystick motoring */
  void run();
  /**
   * Stop joystick control and device */
  void stop();
  /**
   * Test for manuel override, and do RC control if needed
   * \param inManuelControl is current state
   * \returns true if in manuel control */
  bool testJoy(bool inManualControl);
  
public:
  /** is mission overwritten by manual override */
  bool manOverride = false;
  /** Control forward velocity in m/s */
  float velocity = 0.0;
  /** Control curvature in m^-1 */
  float turnVelocity = 0.0;
  /** Servo 2 position (big servo) -1000..1000 */
  float servo2Pos;
  
private:
  /**
   * Open joustick device,
   * \returns false if device not found */
  bool initJoy();
  /**
   * Get fresh data from joystick 
   * \return false if device dissapeared */
  bool getNewJsData();
  /**
   * Decode relevant axis and buttons, and
   * translate to robot commands */
  void joyControl();
  /**
   * Enter state with automatic control */
  void enterAutoState();
  /**
   * Enter state with manual control, and velocity == 0 */
  void enterManualState();
  /**
   * send RC messages to robot */
  void runManualControl();
  /**
   * Shift to manuel RC control */
  void initManualControl();
  /**
   * Shift back to auto mode */
  void stopManualControl();
  
  
private:
  // pointer to regbot interface
  UBridge * bot;
  // camera control
  UCamera * cam;
  // read thread handle
  thread * th1;
  // set true to stop thread
  bool th1stop;
  // device
  const char * joyDevice = "/dev/input/js0";
  int jDev = 0;  ///File descriptors
  volatile bool joyRunning; // flag for device present
  // manual control flag to avoid too mant toggles
  char toggleState = 0;
  /// are we running in fast mode = 1.0, otherwise a bit slower with this factor
  float fastScale = 0.5;
  //Structure to hold joystick values
  struct jVal {
    int button[16];
    int axes[16];
  };
  struct jVal joyValues;
  char number_of_axes, number_of_buttons;
  /** old servo position */
  float servo2Pos_old;
};

#endif
