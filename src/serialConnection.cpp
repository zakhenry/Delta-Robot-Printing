//
//  SerialConnection.cpp
//  Delta Sim
//
//  Created by Zak Henry on 9/06/11.
//  Copyright 2011 __MyCompanyName__. All rights reserved.
//

#include "SerialConnection.h"


SerialConnection::SerialConnection(){ //constructor
    
	cout << "Stepper control class instantiated \n";
    
    stepper0Ready = stepper1Ready = stepper2Ready = true;
    stepper0Connected = stepper1Connected = stepper2Connected = false;
    cout <<"SerialConnection constructor complete\n";

}

void SerialConnection::setupDevices(){
    
    serial0.enumerateDevices();
    printf("enumerated serial0 devices\n");
    
//    if(!serial0.setup(/*"tty.usbserial-A9007Mbm"*/5, 9600)){
    if(!serial0.setup("/dev/tty.usbserial-A4005jpk", 9600)){
        printf("Serial0 setup failed!\n");
    }else{
        stepper0Connected = true;
    }
    
//    if(!serial1.setup(/*"tty.usbserial-A9007Mbm"*/7, 9600)){
    if(!serial1.setup("/dev/tty.usbserial-A4005jpl", 9600)){
        printf("Serial1 setup failed!\n");
    }else{
        stepper1Connected = true;
    }
    
//    if(!serial2.setup(/*"tty.usbserial-A9007Mbm"*/9, 9600)){
    if(!serial2.setup("/dev/tty.usbserial-A4005jpn", 9600)){
        printf("Serial2 setup failed!\n");
    }else{
        stepper2Connected = true;
    }
    
    cout <<"SerialConnection setup complete\n";
}

bool SerialConnection::println(int stepper, string line){
    
    line += "\n";
    
    unsigned char * charLine = new unsigned char[line.size()+1];
    charLine[line.size()]=0;
    memcpy(charLine,line.c_str(),line.size());
    
    serialMessage newMessage;
    
    newMessage.time = ofToString(ofGetHours())+":"+ofToString(ofGetMinutes())+":"+ofToString(ofGetSeconds());
    newMessage.input = false;
    newMessage.message = line;
    
    switch (stepper) {
        case 0:
        {
            if (stepper0Connected){
                stepper0Ready = false;
                serial0.writeBytes(charLine,line.size());
                stepper0io.push_back(newMessage);
                cout <<"output to serial0: "<<charLine<<"\n";
            }
            
        }
        break;
            
        case 1:
        {
            if (stepper1Connected){
                stepper1Ready = false;
                serial1.writeBytes(charLine,line.size());
                stepper1io.push_back(newMessage);
                cout <<"output to serial1: "<<charLine<<"\n";
            }
        }
        break;
            
        case 2:
        {
            if (stepper2Connected){
                stepper2Ready = false;
                serial2.writeBytes(charLine,line.size());
                stepper2io.push_back(newMessage);
                cout <<"output to serial2: "<<charLine<<"\n";
            }
        }
        break;
            
        default:
        break;
    }
}

void SerialConnection::setStepper(int stepper, float angle, float speed){
    
    char buffer [50];
//    sprintf (buffer, "d%f-s%de", angle+90, (int)speed); //note speed is cast to int for now, will be float eventually. angle has 90 added to it so it can be in the range 0-180
    sprintf(buffer, "%f\n%f", angle, speed);
//    sprintf (buffer, "$"); //note speed is cast to int for now, will be float eventually
    println (stepper, buffer);
    
}

void SerialConnection::update(){
    
//    if (!robotReadyForData()&&steppersConnected()){ //only read if a message has been sent to steppers, and all are connected
        read(serial0, 0, bytesReturned0, messageBuffer0, message0);
        read(serial1, 1, bytesReturned1, messageBuffer1, message1);
        read(serial2, 2, bytesReturned2, messageBuffer2, message2);
//    }

}

bool SerialConnection::steppersConnected(){
    
    
    if (stepper0Connected && stepper1Connected && stepper2Connected){
        return true;
    }
    
    return false;
    
}

bool SerialConnection::robotReadyForData(){
    
    
    if (steppersConnected() && stepper0Ready && stepper1Ready && stepper2Ready){
        return true;
    }
    
    return false;
    
}

void SerialConnection::read(ofSerial& serialPort, int stepperNum, unsigned char * bytesReturned, string& messageBuffer, string& message)
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
                
                if (message == "READY"){
                    message = "";
                    cout << "stepper"<<stepperNum<<" is ready\n";
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
                }else {
                    
                    serialMessage newMessage;
                    
                    newMessage.time = ofToString(ofGetHours())+":"+ofToString(ofGetMinutes())+":"+ofToString(ofGetSeconds());
                    newMessage.input = true;
                    newMessage.message = message;
                    
                    switch (stepperNum) {
                        case 0:
                            stepper0io.push_back(newMessage);
                            break;
                            
                        case 1:
                            stepper1io.push_back(newMessage);
                            break;
                            
                        case 2:
                            stepper2io.push_back(newMessage);
                            break;
                            
                        default:
                            break;
                    }
                    
                }
                
                break;
            }
            else 
            {
                if(*bytesReturned != '\r')
                    messageBuffer += *bytesReturned;
            }
//            cout << "  messageBuffer: (" << messageBuffer << ") steppernumber:"<<stepperNum<<"\n";
        }
        
        // clear the message buffer
        memset(bytesReturned,0,NUM_BYTES);
    }
}
