#ifndef UJEVOIS_H
#define	UJEVOIS_H

#include <iostream>
//#include "urun.h"
#include "TimeoutSerial.h"
#include <stdlib.h>

#define SERIAL_PORT "/dev/ttyACM0"
#define BAUDRATE 115200
#define ARUCO "YUYV 640 480 20.0 JeVois DemoArUco" // http://jevois.org/moddoc/DemoArUco/modinfo.html
#define BALL "YUYV 320 240 60.0 JeVois ObjectTracker"
#define GATE "YUYV 320 240 60.0 JeVois FirstPython" //http://jevois.org/moddoc/FirstPython/modinfo.html
#define SHAPE "YUYV 320 240 30.0 JeVois ObjectDetect" //http://jevois.org/moddoc/ObjectDetect/modinfo.html

//maybe SaliencySURF http://jevois.org/moddoc/SaliencySURF/modinfo.html

enum {aruco=1,gate,ball,shape};

using namespace std;
using namespace boost;





class uJevois{
public:
int module;
double x_pos, y_pos,z_pos;
double width, height, depth;
int ID;
TimeoutSerial serialobj;
string fromSerial;

/* Constructor, constructs a object called serialobj of class TimeoutSerial in the uJevois object and connects it. */
uJevois();
/*desctructor */
~uJevois();

/* Update uJevois object from open serialport*/
void updateJevois();

/* pass a pointer to the open serial object, uJevois obejct and the number module wanted */
void changeModule(int module);

/*Connect to Jevois serial port*/
void initJevois();

/*disconnect */
void disconnectJevois();

bool checkStatus();

bool sendStringAndCheck(string payload, string reply);

};

int StringToNumber(const string &Text);

/**/
void getArUco(uJevois *jev);


#endif
