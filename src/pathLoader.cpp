//
//  pathLoader.cpp
//  Delta Sim
//
//  Created by Zak Henry on 9/06/11.
//  Copyright 2011 __MyCompanyName__. All rights reserved.
//

#include "pathLoader.h"


PathLoader::PathLoader(){ //constructor
    
	cout << "Path loader class instantiated \n";
    
    listPathFiles("../../../data", availablePaths);
    
    cout << "There is "<<availablePaths.size()<<" pathFile available (";
    for (int i=0; i<availablePaths.size(); i++){
        cout << availablePaths[i]<<", ";
    }
    cout << ")\n";
    
    changePathFile("test");
}

bool PathLoader::listPathFiles(string dir, vector<string> &pathFiles){ //returns paths of point paths
    
    vector<string>files;
    
    DIR *dp;
    struct dirent *dirp;
    if((dp  = opendir(dir.c_str())) == NULL) {
        cout << "Error(" << errno << ") opening " << dir << endl;
        return errno;
    }
    
    while ((dirp = readdir(dp)) != NULL) {
        files.push_back(string(dirp->d_name));
    }
    closedir(dp);
    
    string fileext = ".drp"; //delta robot paths
    for (int i=0; i<files.size(); i++){
        if (files[i].find(fileext)!=string::npos){ //if string is found
            pathFiles.push_back(files[i]);
        }
    }
    return 0;
}

bool PathLoader::changePathFile(string filename){
    
    pathFile newPathFile;
    
    
    xml.clear();
    if (!xml.loadFile(filename+".drp")){
        cout << "The level \""<<filename<<".drp\" could not be found\n";
        return false;
    }else{
        
        string xmlDump;
        xml.copyXmlToString(xmlDump);
        
//        cout <<"loaded string gives: "<<xmlDump<<"\n";
        
        xml.pushTag("parameters");
            newPathFile.parameters.units = xml.getValue("units", 0);
            newPathFile.parameters.speed = xml.getValue("speed", 0);
        xml.popTag();
        
        xml.pushTag("points");
        for (int i=0; i<xml.getNumTags("point"); i++){
            xml.pushTag("point", i);
                ofPoint newPoint;
                newPoint.x = xml.getValue("x", 0);
                newPoint.y = xml.getValue("y", 0);
                newPoint.z = xml.getValue("z", 0);
                newPathFile.points.push_back(newPoint);
            xml.popTag();
        }
        xml.popTag();
        
        currentPathFile = newPathFile; //should overwrite the current path file
    }
    
}

void PathLoader::drawCurrentPath(bool showPaths){

    
    if (currentPathFile.points.size()>0){
        
        if (showPaths){
            glBegin(GL_LINE_STRIP);
            
            ofSetColor(255, 255, 200);
            
            for (int i=0; i<currentPathFile.points.size(); i++){
                glVertex3f(currentPathFile.points[i].x, currentPathFile.points[i].z, currentPathFile.points[i].y); 
            }
            
            glEnd();
        }
        
        glPointSize(5.0);
        glBegin(GL_POINTS);
        
        ofSetColor(255, 255, 0);
        
        for (int i=0; i<currentPathFile.points.size(); i++){
            glVertex3f(currentPathFile.points[i].x, currentPathFile.points[i].z, currentPathFile.points[i].y);
        }
        
        glEnd();
    }
    
    

    
}