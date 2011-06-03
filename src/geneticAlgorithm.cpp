//
//  geneticAlgorithm.cpp
//  Delta Sim
//
//  Created by Zak Henry on 19/05/11.
//  Copyright 2011 __MyCompanyName__. All rights reserved.
//

#include "geneticAlgorithm.h"



GeneticAlgorithm::GeneticAlgorithm() : deltaRobot(1){ //constructor ( after the : it constructs the deltaRobot)
    
    parms.maxX = 5;
    parms.minX = -5;
    parms.maxY = 5;
    parms.minY = -5;
    parms.maxZ = 5;
    parms.minZ = -5;
    parms.mutationFactor = 0.2; //percentage each phenotype of parent is mutated by
    
    
    populationSize = 100;
    breedingPopulationSize = 0.666;
    generations = 0;
    currentIdNumber = 0;
    
	cout << "Genetic Algorithmic search class initiated \n";
}

void GeneticAlgorithm::run(){
    
    reset(); //reset just in case a rerun is attempted
    cout << "Genetic algorithm running...\n";
    
    initiatePopulation();
    cullPopulation();
/*    
    while (generations<1000) {
        for (int i=0; i<100; i++) {
            rutAndBreedIndividuals();
        }
        
        sortPopulationByFitness();
        cullPopulation();
//        padPopulationWithRandomSpecimens();
    }
*/    
    
    while (generations<100) {
        
        //"popularity function" possibly could have a period of 'socializing' for the population where each specimen is allowed to 'walk' toward the nearest fittest (could be more than one to avoid local optimum finding) individual 
        breedPopulation();
        padPopulationWithRandomSpecimens();
        sortPopulationByFitness();
        cullPopulation();
        removeDuplicatesFromPopulation();
        
        
        cout << "Generation:"<<generations<<" Best Specimen ["<<population[0].idNum<<"]("<<population[0].x<<", "<<population[0].y<<" aged "<<population[0].age<<" with "<<population[0].children<<" children) has fitness "<<population[0].fitness << "\n";
//        cout <<"population size: "<<population.size()<<"\n";
    }
 
    cout << "Best specimen found in generation "<<population[0].generation<<"\n";
    
    
    for (int i=0; i<population.size(); i++){
//        cout << "Last Generation Specimen ["<<population[i].idNum<<"]("<<population[i].x<<", "<<population[i].y<<" aged "<<population[i].age<<" with "<<population[i].children<<" children) has fitness "<<population[i].fitness << "\n";
    }
    
    cout << "GA Finished\n";
}


void GeneticAlgorithm::reset(){
    generations = 0;
    population.clear(); //removes all specimens
    currentIdNumber = 0;
    cout << "GA Reset\n";
}

void GeneticAlgorithm::initiatePopulation(){
    
    int initialPopulation = populationSize;
    
    for (int i=0; i<initialPopulation; i++){
        population.push_back(generateRandomSpecimen(parms));
    }
    
    for (int i=0; i<population.size(); i++){
//        cout << "Initial Population Specimen ["<<population[i].idNum<<"]("<<population[i].x<<", "<<population[i].y<<" aged "<<population[i].age<<" with "<<population[i].children<<" children) has fitness "<<population[i].fitness << "\n";
        
    }
    
//    sortPopulationByFitness();
//    cullPopulation();
    
    cout << "Population initiated\n";
}

GeneticAlgorithm::specimen GeneticAlgorithm::generateRandomSpecimen(parameters parms){
    
    specimen newSpecimen;
    
    do {
    
    newSpecimen.x = ofRandom(parms.minX, parms.maxX);
    newSpecimen.y = ofRandom(parms.minY, parms.maxY);
    newSpecimen.z = ofRandom(parms.minZ, parms.maxZ);
    
    newSpecimen.fitness = deltaRobot.calculateCartesianPointCloudSize(newSpecimen.x, newSpecimen.y, newSpecimen.z, newSpecimen.fitnessTimeCalc); //this is likely going to be the choke point (not actually running point cloud algo yet)
    
//    cout << "new specimen fitness is: ()" <<newSpecimen.fitness<<"\n";
    } while (newSpecimen.fitness==-1);
    
    newSpecimen.age = 0; //any member of population created randomly will have an age of 0
    newSpecimen.generation = generations;
    newSpecimen.children = 0;
    newSpecimen.idNum = nextIdNumber();
    
    return newSpecimen;
}

void GeneticAlgorithm::sortPopulationByFitness(){
    
    for (int i=0; i<population.size(); i++){
//        cout << i<<": Specimen ("<<population[i].x<<", "<<population[i].y<<")has fitness "<<population[i].fitness << "\n";
    }
    
    sort(population.begin(), population.end(), compareFitness()); 

}

void GeneticAlgorithm::cullPopulation(){
    
//    cout << "population before cull = "<<population.size()<<"\n";
    
    int populationToSurvive = breedingPopulationSize*population.size();
    
//    cout << "surviving population count = "<<populationToSurvive<<"\n";
    
    population.erase(population.begin()+populationToSurvive, population.end()); //remove unfit specimens
    
//    cout << "population after cull = "<<population.size()<<"\n";

    for (int i=0; i<population.size(); i++){
        population[i].age++; //increment age
//        cout << i<<": Specimen ("<<population[i].x<<", "<<population[i].y<<") has fitness "<<population[i].fitness << "\n";
    }
    
}

void GeneticAlgorithm::removeDuplicatesFromPopulation(){
    
    vector<specimen>::iterator it = unique(population.begin(), population.end(), compareSpecimenEquality());
    population.resize(it - population.begin());
    
//    cout << "pop size after all culling: "<<population.size()<<"\n";
    
}

//"popularity function" possibly could have a period of 'socializing' for the population where each specimen is allowed to 'walk' toward the nearest fittest (could be more than one to avoid local optimum finding) individual 

void GeneticAlgorithm::breedPopulation(){
    
    vector<specimen> breedingPopulation;
    breedingPopulation = population;
    
    population.clear(); //this is only so the parents can have children counters incremented. delete if too costly
    
    while (breedingPopulation.size()>=2) { //while children can still be made
//    while (population.size()<populationSize){ //use only when not adding new random specimens
        
        //im sure the following could be handled much better? surely??
        
        int firstParentId = ofRandom(0, breedingPopulation.size()-1);
        specimen firstParent = breedingPopulation[firstParentId];
        breedingPopulation.erase(breedingPopulation.begin()+firstParentId);
        firstParent.children ++;
        
        int secondParentId = ofRandom(0, breedingPopulation.size()-1);
        specimen secondParent = breedingPopulation[secondParentId];
        breedingPopulation.erase(breedingPopulation.begin()+secondParentId);
        secondParent.children ++;
        
        
        specimen child = createChild(firstParent, secondParent);
        
        population.push_back(firstParent);
        population.push_back(secondParent);
        population.push_back(child);
        
    }
    
    for (int i=0; i<population.size(); i++){
//        cout << i<<": Specimen ("<<population[i].x<<", "<<population[i].y<<") has fitness "<<population[i].fitness << "\n";
    }
    
    generations ++; //a new generation has been added
    
    
}

void GeneticAlgorithm::rutAndBreedIndividuals(){
    
    //uses stochastic universal sampling to select individuals who are sufficiently far apart from each other (minimises neighbouring mating) [should i sort pop by fitness before? expensive?]
    int currPopulationSize = population.size(); //calculate once for this round
    int susIncrement = currPopulationSize/3;
    
    int randomSpecimenId = ofRandom(0, currPopulationSize); //30 //could make second parm static for speed? if so would break down if pop size is dynamic
    int candidate1Id = randomSpecimenId; //24
    int candidate2Id = (candidate1Id+susIncrement)>currPopulationSize ? candidate1Id+susIncrement-currPopulationSize : candidate1Id+susIncrement;
    int candidate3Id = (candidate2Id+susIncrement)>currPopulationSize ? candidate2Id+susIncrement-currPopulationSize : candidate2Id+susIncrement;
    
    vector<specimen>tournament;
    
    //add candidates to tournament
    
    tournament.push_back(population[candidate1Id]);
    tournament.push_back(population[candidate2Id]);
    tournament.push_back(population[candidate3Id]);
    
    //erase candidates from original population
    
    population.erase(population.begin()+candidate1Id);
    population.erase(population.begin()+candidate2Id-1);
    population.erase(population.begin()+candidate3Id-2);
    
    //fight!
    sort(tournament.begin(), tournament.end(), compareFitness());
    
    //make babies
    
    specimen child = createChild(tournament[0], tournament[1]);
    
    //put the winners back in to population
    population.push_back(tournament[0]); 
    population.push_back(tournament[1]);
    population.push_back(child);
    
//    cout << "Child Specimen ["<<child.idNum<<"]("<<child.x<<", "<<child.y<<" aged "<<child.age<<" with "<<child.children<<" children) has fitness "<<child.fitness << "\n";
    
    
//    cout <<"candidate1id: "<<candidate1Id<<", candidate2id: "<<candidate2Id<<", candidate2id: "<<candidate3Id<<"\n";
    
    generations ++; //a new "generation" has been added (means completely different thing to what it did in previous setup - generation is incremented at every birth

}


GeneticAlgorithm::specimen GeneticAlgorithm::createChild(specimen parentA, specimen parentB){ //currently using two parents though three is apparently better in some cases if a little weird
    //defines crossover algorithm (currently using simple averaging of vars)
    
    specimen child;
    
    /*Crossover and Mutate method (better at finding multiple solutions)*/
///*    
    //crossover
    child.x = parentA.x;
    child.y = parentB.y;
    
    child.z = 0;
    
    //mutation
    child.x *= ofRandom(1-parms.mutationFactor, 1+parms.mutationFactor);
    child.y *= ofRandom(1-parms.mutationFactor, 1+parms.mutationFactor);
//*/    
    /*Average method (better at finding one maximum fast. Could possibly get more easily stuck on a local max if insufficient randomness of children and padding)*/
/*    
    child.x = (parentA.x+parentB.x)*0.5;
    child.y = (parentA.y+parentB.y)*0.5;
*/    
    //fitness test
    
    child.fitness = deltaRobot.calculateCartesianPointCloudSize(child.x, child.y, child.z, child.fitnessTimeCalc);
    
    //assign basic parms
    
    child.age = 0;
    child.generation = generations;
    child.children = 0;
    child.idNum = nextIdNumber();
    
//    cout <<"offspring from parents ("<<parentA.x<<","<<parentA.y<<")["<<parentA.fitness<<"] and ("<<parentB.x<<","<<parentB.y<<")["<<parentB.fitness<<"] results in child ("<<child.x<<","<<child.y<<")["<<child.fitness<<"]\n";
    
    return child;
}

void GeneticAlgorithm::padPopulationWithRandomSpecimens(){
    
    //fills population's remaining space with complete random specimens to keep the current colony on its toes
    
    while (population.size()<populationSize) {
        population.push_back(generateRandomSpecimen(parms));
    }
    
    /*for (int i=0; i<population.size(); i++){
        cout << "Generation:"<<generations<<" Specimen ["<<population[i].idNum<<"]("<<population[i].x<<", "<<population[i].y<<" aged "<<population[i].age<<" with "<<population[i].children<<" children) has fitness "<<population[i].fitness << "\n";
    }*/
        
}

int GeneticAlgorithm::nextIdNumber(){
    currentIdNumber++;
    return currentIdNumber;
}



vector<GeneticAlgorithm::specimen>GeneticAlgorithm::bruteForceSearchSpace(parameters parms){
    
    searchIncrement = 0.1;
    vector<specimen>specimens;
    
    int idNum = 0;
    
    for (float xVal = parms.minX; xVal<parms.maxX; xVal+=searchIncrement) {
        for (float yVal = parms.minY; yVal<parms.maxY; yVal+=searchIncrement) {
            specimen newSpecimen;
            newSpecimen.x = xVal;
            newSpecimen.y = yVal;
            newSpecimen.z = 0;
            
            newSpecimen.fitness = deltaRobot.calculateCartesianPointCloudSize(newSpecimen.x, newSpecimen.y, newSpecimen.z, newSpecimen.fitnessTimeCalc); //this is likely going to be the choke point (not actually running point cloud algo yet)
            newSpecimen.age = 0; //any member of population created randomly will have an age of 0
            newSpecimen.generation = 0;
            newSpecimen.children = 0;
            newSpecimen.idNum = idNum++;
            
            if (newSpecimen.fitness!=-1){ //specimen is inside plausible search space
                specimens.push_back(newSpecimen);
            }
            
        }
    }
    
    return specimens;
    
}

void GeneticAlgorithm::calculateSearchSpace(){
    if (allSpecimens.size()>0){
        allSpecimens.clear();
    }
        
    allSpecimens = bruteForceSearchSpace(parms);
    
    cout <<"Search space brute forced\n";
    
    for (int i=0; i<allSpecimens.size(); i++){
//        cout << "Brute forced specimen ["<<allSpecimens[i].idNum<<"]("<<allSpecimens[i].x<<", "<<allSpecimens[i].y<<" aged "<<allSpecimens[i].age<<" with "<<allSpecimens[i].children<<" children) has fitness "<<allSpecimens[i].fitness <<" calculation took "<<allSpecimens[i].fitnessTimeCalc<<" seconds\n";
        
    }

}

void GeneticAlgorithm::drawSearchSpace(){
    
    glPushMatrix();
    
    
    glTranslatef(ofGetWidth()/2,ofGetHeight()/2-400,0); //moves coordinates to centre (ish) of scene
    
    if (allSpecimens.size()>0){
        
        glPointSize(2.0);
        glBegin(GL_POINTS);
//        ofSetColor(255, 255, 255);
        
        ofColor color;
        color.r = 255;
        color.g = 0;
        color.b = 255;
        
        for (int i=0; i<allSpecimens.size(); i++){
            
            ofColor newColor = HSVToRGB(allSpecimens[i].fitness/400, 0.5, 1, color);
            
            ofSetColor(newColor.r, newColor.g, newColor.b);
            
            glVertex3f(allSpecimens[i].x*100, allSpecimens[i].fitness/2, allSpecimens[i].y*100);
        }
        
//        cout << population.size() << "\n";
        
        for (int i=0; i<population.size(); i++){
            
            ofColor newColor = HSVToRGB(population[i].fitness/500, 1, 1, color);
            
            ofSetColor(newColor.r, newColor.g, newColor.b);
            
            glVertex3f(population[i].x*100, population[i].fitness/2, population[i].y*100);
        }
        
        
        
        glEnd();
    }
    
    glPopMatrix();
}

void GeneticAlgorithm::drawCurrentPopulation(){
    
    glPushMatrix();
    
    
    glTranslatef(ofGetWidth()/2,ofGetHeight()/2-400,0); //moves coordinates to centre (ish) of scene
    
    if (allSpecimens.size()>0){
        
        glPointSize(5.0);
        glBegin(GL_POINTS);
        //        ofSetColor(255, 255, 255);
        
        ofColor color;
        color.r = 255;
        color.g = 0;
        color.b = 255;
        
        for (int i=0; i<population.size(); i++){
            
            ofColor newColor = HSVToRGB(population[i].fitness/500, 1, 1, color);
            
            ofSetColor(newColor.r, newColor.g, newColor.b);
            
            glVertex3f(population[i].x*100, population[i].fitness/2, population[i].y*100);
        }
        
        
        
        glEnd();
    }
    
    glPopMatrix();
}


ofColor GeneticAlgorithm::HSVToRGB(float H, float S, float V, ofColor &in){ // (0-1, 0-1, 0-1) //applies hsv transform to the ofColor &in

    H = H*360;
    
    float VSU = V*S*cos(H*M_PI/180);
    float VSW = V*S*sin(H*M_PI/180);
    
    ofColor ret;
    ret.r = (.299*V+.701*VSU+.168*VSW)*in.r + (.587*V-.587*VSU+.330*VSW)*in.g + (.114*V-.114*VSU-.497*VSW)*in.b;
    ret.g = (.299*V-.299*VSU-.328*VSW)*in.r + (.587*V+.413*VSU+.035*VSW)*in.g + (.114*V-.114*VSU+.292*VSW)*in.b;
    ret.b = (.299*V-.3*VSU+1.25*VSW)*in.r + (.587*V-.588*VSU-1.05*VSW)*in.g + (.114*V+.886*VSU-.203*VSW)*in.b;
    return ret;
}
