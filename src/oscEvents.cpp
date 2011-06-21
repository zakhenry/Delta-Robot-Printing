/*
 *  oscEvents.cpp
 *  Delta Sim:  OSC transport events for Delta Robot
 *
 *  Created by Tarei King on 8/04/11.
 *  Copyright 2011 Tarei King. All rights reserved.
 *
 *  Built with ofxOSC library at https://github.com/fractaloop/ofxOsc.git
 *  And.. it has blob support, woot!
 */

#include "oscEvents.h"

oscEvents::oscEvents(){
    cout << "listening for osc messages on port " << PORT << "\n";
	receiver.setup( PORT );
    highpoint.set(0,0,0);
    
}

void oscEvents::update(){
    // hide old messages
	for ( int i=0; i<NUM_MSG_STRINGS; i++ )
	{
		if ( timers[i] < ofGetElapsedTimef() )
			msg_strings[i] = "";
	}
    
    // check for waiting messages
	while( receiver.hasWaitingMessages() )
	{
		// get the next message
		ofxOscMessage m;
		receiver.getNextMessage( &m );
        
        // recieve the normalised highpoint data
		if ( m.getAddress() == "/delta/highpoint" )
		{
            float x, y, z;
			// both the arguments are int32's
			x = m.getArgAsFloat( 0 );
			y = m.getArgAsFloat( 1 );
            z = m.getArgAsFloat( 2 );
            
            setHighpoint(x, y, z);
            cout << "delta highpoint received as: " << highpoint.z;
            
		}
        
        // implement point cloud listener
		else if ( m.getAddress() == "/delta/cloud" )
		{
            
		}
        
        // get blob data if blobs -- can implement later
        else if( m.getAddress() == "/delta/blob" )
        {
            
        }
        
	}
    
}

ofPoint oscEvents::setHighpoint (float _x, float _y, float _z){
    highpoint.set(_x, _y, _z);
}

ofPoint oscEvents::getHighpoint(){
    return highpoint;
}