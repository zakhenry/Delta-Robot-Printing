//
//  stepperControl.cpp
//  Delta Sim
//
//  Created by Zak Henry on 9/06/11.
//  Copyright 2011 __MyCompanyName__. All rights reserved.
//

#include "stepperControl.h"


StepperControl::StepperControl(){ //constructor
    
	cout << "Stepper control class instantiated \n";
    
    stepper0Ready = stepper1Ready = stepper2Ready = true;
    stepper0Connected = stepper1Connected = stepper2Connected = false;


}

void StepperControl::setupDevices(){
    
    serial0.enumerateDevices();
    printf("enumerated serial0 devices\n");
    
    if(!serial0.setup(/*"tty.usbserial-A9007Mbm"*/5, 9600)){
        printf("Serial setup failed!\n");
    }else{
        stepper0Connected = true;
    }
    
    if(!serial1.setup(/*"tty.usbserial-A9007Mbm"*/7, 9600)){
        printf("Serial setup failed!\n");
    }else{
        stepper1Connected = true;
    }
    
    if(!serial2.setup(/*"tty.usbserial-A9007Mbm"*/9, 9600)){
        printf("Serial setup failed!\n");
    }else{
        stepper2Connected = true;
    }
}

bool StepperControl::println(int stepper, string line){
    
    unsigned char * charLine = new unsigned char[line.size()+1];
    charLine[line.size()]=0;
    memcpy(charLine,line.c_str(),line.size());
    
    switch (stepper) {
        case 0:
        {
            if (stepper0Connected){
                stepper0Ready = false;
                serial0.writeBytes(charLine,line.size());
//                serial0.flush();
                cout <<"output to serial0: "<<charLine<<"\n";
            }
            
        }
        break;
            
        case 1:
        {
            if (stepper1Connected){
                stepper1Ready = false;
                serial1.writeBytes(charLine,line.size());
                cout <<"output to serial1: "<<charLine<<"\n";
            }
        }
        break;
            
        case 2:
        {
            if (stepper2Connected){
                stepper2Ready = false;
                serial2.writeBytes(charLine,line.size());
                cout <<"output to serial2: "<<charLine<<"\n";
            }
        }
        break;
            
        default:
        break;
    }
}

void StepperControl::setStepper(int stepper, float angle, float speed){
    
    char buffer [50];
    sprintf (buffer, "d%f-s%de", angle+90, (int)speed); //note speed is cast to int for now, will be float eventually. angle has 90 added to it so it can be in the range 0-180
//    sprintf (buffer, "$"); //note speed is cast to int for now, will be float eventually
    println (stepper, buffer);
    
}

void StepperControl::update(){
    
    read(serial0, 0);
    read(serial1, 1);
//    read(serial2, 2);

}

bool StepperControl::readUntil(int stepper, string& rResult, char cUntil) {
    cout <<"waiting for message from "<<stepper<<"\n";
//	char b[1];
    unsigned char * b = new unsigned char[1];
	char buf[1];
	int  i = 0;
    
	do {
        int n = -1;
		
        switch (stepper) {
            case 0:
                n = serial0.readBytes(b, 1);  // read a char at a time
//                serial0.flush();
                
                break;
                
            case 1:
                n = serial1.readBytes(b, 1);  // read a char at a time
                break;
                
            case 2:
                n = serial2.readBytes(b, 1);  // read a char at a time
                break;
                
            default:
                break;
        }
		if( n == -1) return false;    // couldn't read
		if(n == 0) {
			return false;
		}
		buffer.push_back(b[0]);
        
		i++;
        
//        cout <<"Serial byte is "<<b[0]<<"\n";
	} while( b[0] != cUntil );
    
    cout <<"Buffer is ";
    for (int j=0; j<buffer.size(); j++){
        cout <<buffer[j];
    }
    cout <<"\n";
	
	std::vector<char>::iterator it = buffer.begin();
	while(it != buffer.end()) {
		rResult.push_back((*it));
		++it;
	}
	buffer.clear();
	return true;
    
}

bool StepperControl::readBytes(int stepper, string& rResult, int bytesToRead) {
    cout <<"waiting for message from "<<stepper<<"\n";
    //	char b[1];
    unsigned char * b = new unsigned char[bytesToRead];
    
    int n = -1;
    
        switch (stepper) {
            case 0:
                n = serial0.readBytes(b, bytesToRead);  // read a char at a time
                
                cout <<"buffer reads: "<<b<<"\n";
                
                if (n>0){
                    return true;
                }
                return false;
                
                break;
                
            case 1:
                n = serial1.readByte();  // read a char at a time
                break;
                
            case 2:
                n = serial2.readByte();  // read a char at a time
                break;
                
            default:
                break;
        }
    
    
    memcpy(b,rResult.c_str(),bytesToRead);
    
}

bool StepperControl::robotReadyForData(){
    
    
    if (stepper0Connected && stepper1Connected && /*stepper2Connected &&*/ stepper0Ready && stepper1Ready /*&& stepper2Ready*/){
        return true;
    }
    
    return false;
    
}

void StepperControl::read(ofSerial& serialPort, int stepperNum)
{
    
    // if we've got new bytes
    if(serialPort.available() > 0)
    {
        // we wil keep reading until nothing is left
        while (serialPort.available() > 0)
        {
            
            // we'll put the incoming bytes into bytesReturned
            serialPort.readBytes(bytesReturned, NUM_BYTES);
            
            // if we find the splitter we put all the buffered messages 
            //   in the final message, stop listening for more data and 
            //   notify a possible listener
            // else we just keep filling the buffer with incoming bytes. 
            if(*bytesReturned == '\n')
            {
                message = messageBuffer;
                messageBuffer = "";
                
                if (message == "done"){
                    cout << "stepper"<<stepperNum<<" is done";
                    switch (stepperNum) {
                        case 0:
                            stepper0Ready = true;
                            break;
                            
                        case 1:
                            stepper1Ready = true;
                            break;
                            
                        case 2:
                            stepper2Ready = true;
                            break;
                            
                        default:
                            break;
                    }
                }
                cout << "new message is: "<<message<<"\n";
                
                break;
            }
            else 
            {
                if(*bytesReturned != '\r')
                    messageBuffer += *bytesReturned;
            }
            //cout << "  messageBuffer: " << messageBuffer << "\n";
        }
        
        // clear the message buffer
        memset(bytesReturned,0,NUM_BYTES);
    }
}
