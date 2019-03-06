#include "ujevois.h"
#include <string>


/*constructor*/
uJevois::uJevois()
{
int module;
double x_pos=0;
double y_pos=0;
int ID;
TimeoutSerial serialobj;
string fromSerial;
};


/*destructor*/
uJevois::~uJevois(){
	this->serialobj.close();
};


void uJevois::initJevois(){
	this->serialobj.open(SERIAL_PORT,BAUDRATE); //connect to serialport
    this->serialobj.setTimeout(posix_time::seconds(5)); //set timeout

    if(this->checkStatus()){ //Check if jevois is online

    	//this->sendStringAndCheck("setpar serlog DBG","OK");
    	sleep(0);
    	this->sendStringAndCheck("setpar serout USB","OK");
    	sleep(0);
    	this->sendStringAndCheck("getpar serout","engine:serout USB");
    	//this->serialobj.writeString("setpar serlog DBG"); //setting logging to Debug. set INF for info, ERR for error
    	//this->serialobj.writeString("setpar serout USB"); //setting serial output to USB. HARD for four wire output
    	//this->serialobj.writeString("getpar serlog");
    	//this->fromSerial = this->serialobj.readStringUntil("\n");
    	//cout<<this->fromSerial<<"\n"; //debug

    }
    else{
    	cout<<"camera not online";
    }
//    std::cout<<this->serialobj.readStringUntil("\n")<<endl;
}



void uJevois::disconnectJevois(){
    this->serialobj.close();
}

void uJevois::changeModule(int module){
this->module = module;
switch(module){
case aruco:
	this->sendStringAndCheck(ARUCO,"OK");
	this->sendStringAndCheck("streamon","OK");
break;
case gate:
	this->sendStringAndCheck(GATE,"OK");
	this->sendStringAndCheck("streamon","OK");
break;
case ball:
	this->sendStringAndCheck(BALL,"OK");
	this->sendStringAndCheck("streamon","OK");
break;
case shape:
	this->sendStringAndCheck(SHAPE,"OK");
	this->sendStringAndCheck("streamon","OK");
break;
};
}


void uJevois::updateJevois(){
	switch(this->module){
	case aruco:
	{
		this->fromSerial=this->serialobj.readStringUntil("\n");
		string IDchar=this->fromSerial.substr(this->fromSerial.rfind("id:"),2);
		this->ID=StringToNumber(IDchar);
		this->fromSerial.rfind("x:");
		this->fromSerial.rfind("y:");
		this->fromSerial.rfind("w:");
		this->fromSerial.rfind("h:");
		//doing something, parsing serial output
		//return the ID of aruco marker and position
	}
	break;
	case gate:
		//doing something, parsing serial output
		//position of gate
	break;
	case ball:
		//doing something, parsing serial output

	break;
	case shape:
		//doing something, parsing serial output
		//return the shape and position
	break;
	};
}

bool uJevois::checkStatus(){
    bool ok = false;
    string reply;
	this->serialobj.writeString("ping\n"); //check if camera is running
    reply = this->serialobj.readString(5); //Reading output from camera
    if(reply=="ALIVE"){ //if read string is ALIVE
    ok = true;
    cout<<"Connected to serialport and jevois is alive\n";
    }
    else{ ok = false;};
    return ok;
}

bool uJevois::sendStringAndCheck(string payload, string wantedreply){
    bool ok=false;
	string reply;
	//cout << payload<<"\n";
    	this->serialobj.writeString(payload + "\n");
    	reply = this->serialobj.readString(wantedreply.length()+2);
    	//cout << reply;
    	//size_t found=reply.rfind("OK");
    	if(reply.rfind(wantedreply) != reply.npos){
    		cout<<"String sent and confirmed\n";
    		ok=true;
    	}
    	else ok=false;
    	cout<<"Send string failed. sent: "<<payload<<" ,and ecspected "<< wantedreply<<" but recieved "<<reply<<"\n";

    return ok;
}


  int StringToNumber ( const string &Text )
  {
     istringstream ss(Text);
     int result;
     return ss >> result ? result : 0;
  }

