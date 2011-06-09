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