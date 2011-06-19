/*
 *  oscEvents.h
 *  Delta Sim:  OSC transport events for Delta Robot
 *
 *  Created by Tarei King on 8/04/11.
 *  Copyright 2011 Tarei King. All rights reserved.
 *
 */

#ifndef _OSC_EVENTS // if this class hasn't been defined, the program can define it
#define _OSC_EVENTS // by using this if statement you prevent the class to be called more 
// than once which would confuse the compiler

#include "ofMain.h"
#include "ofxOsc.h"

#define PORT 6969
#define NUM_MSG_STRINGS 20

class oscEvents {
public:
    oscEvents(); //constructor

    void update();
    ofPoint setHighpoint(float _x, float _y, float _z);
    ofPoint getHighpoint();
    
    // ivars
    ofPoint highpoint;

private:
    ofxOscReceiver	receiver;
    
    int				current_msg_string;
    string		msg_strings[NUM_MSG_STRINGS];
    float			timers[NUM_MSG_STRINGS];

    
};


#endif