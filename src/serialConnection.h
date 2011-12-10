//
//  SerialConnection.h
//  Delta Sim
//
//  Created by Zak Henry on 9/06/11.
//  Copyright 2011 __MyCompanyName__. All rights reserved.
//
#ifndef _SERIAL_CONNECTION // if this class hasn't been defined, the program can define it
#define _SERIAL_CONNECTION // by using this if statement you prevent the class to be called more 
// than once which would confuse the compiler

#include "ofMain.h"

#define NUM_BYTES 1

struct serialMessage {
    string time;
    string message;
    bool input;
};

class SerialConnection {
    
    vector<char>buffer;
    
    bool stepper0Ready, stepper1Ready, stepper2Ready;
    bool stepper0Connected, stepper1Connected, stepper2Connected;
    
public:
    
    SerialConnection(); //constructor
    void setupDevices();
    
    ofSerial serial0, serial1, serial2;
    
    bool println(int stepper, string line);
    
    void setStepper(int stepper, float angle, float speed);
    
    void update();
    
    bool steppersConnected();
    bool robotReadyForData();
    
    string message0, message1, message2, messageBuffer0, messageBuffer1, messageBuffer2;
    unsigned char bytesReturned0[1], bytesReturned1[1], bytesReturned2[1];
    void read(ofSerial& serialPort, int portNum, unsigned char * bytesReturned, string& messageBuffer, string& message);
    
    vector<serialMessage>stepper0io, stepper1io, stepper2io;
    
};


#endif 