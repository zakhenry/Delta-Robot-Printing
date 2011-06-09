//
//  pathLoader.h
//  Delta Sim
//
//  Created by Zak Henry on 9/06/11.
//  Copyright 2011 __MyCompanyName__. All rights reserved.
//

#ifndef _PATH_LOADER // if this class hasn't been defined, the program can define it
#define _PATH_LOADER // by using this if statement you prevent the class to be called more 
// than once which would confuse the compiler

#include "ofMain.h"
#include "ofxXmlSettings.h"


class PathLoader {
    
    ofxXmlSettings xml;
    vector<string>availablePaths;
    
    public:
    
    struct parms {
        string units;
        float speed;
    };
    
    struct pathFile {
        parms parameters;
        vector<ofPoint>points;
    };
    
    pathFile currentPathFile;
    
    PathLoader(); //constructor
    
    bool changePathFile(string filename);
    bool listPathFiles(string dir, vector<string> &pathFiles);
    void drawCurrentPath(bool showPaths);
    
};



#endif 