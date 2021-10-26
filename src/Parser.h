/* 
 * Problem file parser -- RAGE Plan Single agent version
 * 
 * By JCS, DFKI-NI 2021.
 *
 */


#ifndef PARSER_H
#define PARSER_H

#include <fstream>
#include <iostream>
#include <iomanip>
#include "simulator.h"
#include "drone.h"
#include "cellar.h"

using std::cout;
using std::endl;
using std::string;

namespace PARSER{
    
    struct COMMAND_LINE{
        string help;
        string problem = "none";
        string inputFile = "none";
        string outputFile = "output.txt";
        int size;
        int number;
        int number2;
        int number3;
        int minDoubles;
        int maxDoubles;
        int numSteps;
        int timeout = 1000;
        int runs = 1;
        int verbose = 0;
        int treeKnowledge = 1;
        int rolloutKnowledge = 1;
        bool fTable = 0;
    };
    
    void parseCommandLine(char ** argv, int argc, COMMAND_LINE& cl){        
        string param, value;
        for(int i=1; i<argc; i+=2){
            param = argv[i];
            
            if(argc > i+1) value = argv[i+1];
            if(param == "--help"){
                cout << "RAGE: Relevance-Aware GEnerative Planning" << endl;
                cout << "Parameters" << endl;
                cout << std::setw(3) << "";
                cout << std::left << std::setw(20) << "--problem";
                cout << std::left << std::setw(100) << "Problem to run" << endl;
                
                cout << std::setw(3) << "";
                cout << std::left << std::setw(20) << "--inputFile";
                cout << std::left << std::setw(100) <<"Problem specification file" << endl;
                
                cout << std::setw(3) << "";
                cout << std::left << std::setw(20) << "--outputFile";
                cout << std::left << std::setw(100) << "Summary output file" << endl;
                
                cout << std::setw(3) << "";
                cout << std::left << std::setw(20) <<"--size";
                cout << std::left << std::setw(100) << "Grid size (problem specific)" << endl;
                
                cout << std::setw(3) << "";
                cout << std::left << std::setw(20) << "--number";
                cout << std::left << std::setw(100) << "No. of elements (problem specific)" << endl;
                
                cout << std::setw(3) << "";
                cout << std::left << std::setw(20) << "--number2";
                cout << std::left << std::setw(100) << "No. of elements (problem specific)" << endl;
                
                cout << std::setw(3) << "";
                cout << std::left << std::setw(20) << "--number3";
                cout << std::left << std::setw(100) << "No. of elements (problem specific)" << endl;
                
                cout << std::setw(3) << "";
                cout << std::left << std::setw(20) << "--minDoubles";
                cout << std::left << std::setw(100) << "Min. amount of simulations (power of 2)" << endl;
                
                cout << std::setw(3) << "";
                cout << std::left << std::setw(20) << "--maxDoubles";
                cout << std::left << std::setw(100) << "Max. amount of simulations (power of 2)" << endl;
                
                cout << std::setw(3) << "";
                cout << std::left << std::setw(20) << "--numSteps";
                cout << std::left << std::setw(100) << "Max. steps before termination" << endl;
                
                cout << std::setw(3) << "";
                cout << std::left << std::setw(20) << "--timeout";
                cout << std::left << std::setw(100) << "Time (seconds) before experiment termination" << endl;
                
                cout << std::setw(3) << "";
                cout << std::left << std::setw(20) << "--runs";
                cout << std::left << std::setw(100) << "No. of episodes" << endl;
                
                cout << std::setw(3) << "";
                cout << std::left << std::setw(20) << "--verbose";
                cout << std::left << std::setw(100) << "Verbosity level (0 = minimal)"  << endl;
                
                cout << std::setw(3) << "";
                cout << std::left << std::setw(20) << "--rolloutKnowledge";
                cout << std::left << std::setw(100) << "Type of Rollout policy (0=Pure, 1=Legal, 2=Smart, 3=PGS)" << endl;
                
                exit(0);
            }
            if(param == "--about"){
                cout << "RAGE Plan v0.2" << endl;
                cout << "By Juan Carlos Saborio-Morales" << endl;
                cout << "2016-2021, DFKI-Niedersachen & Uni-Osnabrueck" << endl;
                exit(0);
            }
            
            if(param == "--problem")
                cl.problem = value;
            else if(param == "--inputFile")
                cl.inputFile = value;
            else if(param == "--outputFile")
                cl.outputFile = value;
            else if(param == "--size")
                cl.size = stoi(value);
            else if(param == "--number")
                cl.number = stoi(value);
            else if(param == "--number2")
                cl.number2 = stoi(value);
            else if(param == "--number3")
                cl.number3 = stoi(value);
            else if(param == "--minDoubles")
                cl.minDoubles = stoi(value);
            else if(param == "--maxDoubles")
                cl.maxDoubles = stoi(value);
            else if(param == "--numSteps")
                cl.numSteps = stoi(value);
            else if(param == "--timeout")
                cl.timeout = stoi(value);
            else if(param == "--runs")
                cl.runs = stoi(value);
            else if(param == "--verbose")
                cl.verbose = stoi(value);
            else if(param == "--treeKnowledge")
                cl.treeKnowledge = stoi(value);
            else if(param == "--rolloutKnowledge")
                cl.rolloutKnowledge = stoi(value);
            else if(param == "--fTable")
                cl.fTable = stoi(value);
            else
                cout << "Unrecognized parameter \"" << param << "\"" << endl;
        }
        
    }
    
    bool parseCellar(PROBLEM_PARAMS* problem_params, std::ifstream& infile){
        cout << "Parsing cellar file" << endl;
        
        problem_params = new CELLAR_PARAMS();
        CELLAR_PARAMS * cellar_params = safe_cast<CELLAR_PARAMS*>(problem_params);
        problem_params->problem = "cellar";
        
        cout << "reading params" << endl;
        
        string param, s_value;
        while(infile >> param >> s_value){
            cout << param << " = " << s_value << endl;
            if(param == "size")
                cellar_params->size = stoi(s_value);
            else if(param == "bottles")
                cellar_params->bottles = stoi(s_value);
            else if(param == "crates")
                cellar_params->crates = stoi(s_value);
            else if(param == "shelves")
                cellar_params->shelves = stoi(s_value);
            else if(param == "discount")
                cellar_params->discount = stof(s_value);
            else if(param == "fDiscount")
                cellar_params->fDiscount = stof(s_value);
            else if(param == "entropy")
                cellar_params->entropy = stof(s_value);
            else if(param == "activation")
                cellar_params->activation = stof(s_value);
            else if(param == "PGSAlpha")
                cellar_params->PGSAlpha = stof(s_value);
            else if(param == "transitionRate")
                cellar_params->transitionRate = stof(s_value);
            else
                cout << "\tWarning: \"" << param << "\" is not a valid parameter." << endl;
        }
        
        cellar_params->description = "cellar[" + std::to_string(cellar_params->size)
                                   + ", " + std::to_string(cellar_params->bottles)
                                   + ", " + std::to_string(cellar_params->crates)
                                   + ", " + std::to_string(cellar_params->shelves)
                                   + "]";
        return true;
    }
    
    bool parseDrone(PROBLEM_PARAMS* problem_params, std::ifstream& infile){
        problem_params = new DRONE_PARAMS();
        DRONE_PARAMS& drone_params = safe_cast<DRONE_PARAMS&>(*problem_params);
        drone_params.problem = "drone";
        
        string param, s_value;
        while(infile >> param >> s_value){
            if(param == "size")
                drone_params.size = stoi(s_value);
            else if(param == "creatures")
                drone_params.creatures = stoi(s_value);
            else if(param == "trees")
                drone_params.trees = stoi(s_value);
            else if(param == "targets")
                drone_params.targets = stoi(s_value);
            else if(param == "activation")
                drone_params.activation = stof(s_value);
            else if(param == "maxPhotos")
                drone_params.maxPhotos = stoi(s_value);
            else if(param == "photos")
                drone_params.photos = stoi(s_value);
            else if(param == "discount")
                drone_params.discount = stof(s_value);
            else if(param == "fDiscount")
                drone_params.fDiscount = stof(s_value);
            else if(param == "recognition")
                drone_params.recognition = stof(s_value);
            else if(param == "moving")
                drone_params.moving = stof(s_value);
            else if(param == "entropy")
                drone_params.entropy = stof(s_value);
            else
                cout << "\tWarning: \"" << param << "\" is not a valid parameter." << endl;
        }        
        drone_params.description = "drone[" + std::to_string(drone_params.size)
                                   + ", " + std::to_string(drone_params.creatures)
                                   + ", " + std::to_string(drone_params.trees)
                                   + "]";
        return true;
    }        
    
    bool parseProblemFile(PROBLEM_PARAMS* problem_params, string inputFile){
        std::ifstream infile(inputFile);
        string param;
        string s_value;

        //FIREFIGHT_PARAMS& params = safe_cast<FIREFIGHT_PARAMS&>(problem_params);
        
        if(!infile.is_open()){
            cout << "Could not open file \"" << inputFile << "\"." << endl;
            return false;
        }
        
        infile >> param >> s_value; //Read first line
        if(param != "problem"){
            cout << "Error.  First line must contain problem type." << endl;
            return false;
        }
        
        bool status;
        
        if(s_value == "cellar") status = parseCellar(problem_params, infile);
        else if(s_value == "drone") status = parseDrone(problem_params, infile);
        else{
            cout << "Problem not recognized." << endl;
            status = false;
        }
                
        infile.close();
        cout << "Finished Parsing" << endl;
        return status;
    }
    
    bool parseCellarFile(CELLAR_PARAMS& cellar_params, string inputFile){
        std::ifstream infile(inputFile);
        string param;
        string s_value;

        if(!infile.is_open()){
            cout << "Could not open file \"" << inputFile << "\"." << endl;
            return false;
        }

        while(infile >> param >> s_value){
            //cout << param << " = " << s_value << endl;
            if(param == "problem")
                cellar_params.problem = s_value;
            else if(param == "size")
                cellar_params.size = stoi(s_value);
            else if(param == "bottles")
                cellar_params.bottles = stoi(s_value);
            else if(param == "crates")
                cellar_params.crates = stoi(s_value);
            else if(param == "shelves")
                cellar_params.shelves = stoi(s_value);
            else if(param == "discount")
                cellar_params.discount = stof(s_value);
            else if(param == "fDiscount")
                cellar_params.fDiscount = stof(s_value);
            else if(param == "entropy")
                cellar_params.entropy = stof(s_value);
            else if(param == "activation")
                cellar_params.activation = stof(s_value);
            else if(param == "PGSAlpha")
                cellar_params.PGSAlpha = stof(s_value);
            else if(param == "transitionRate")
                cellar_params.transitionRate = stof(s_value);
            else
                cout << "\tWarning: \"" << param << "\" is not a valid parameter." << endl;
        }
        infile.close();
        
        cellar_params.description = "cellar[" + std::to_string(cellar_params.size)
                                   + ", " + std::to_string(cellar_params.bottles)
                                   + ", " + std::to_string(cellar_params.crates)
                                   + ", " + std::to_string(cellar_params.shelves)
                                   + "]";
        return true;
    }

    bool parseDroneFile(DRONE_PARAMS& drone_params, string inputFile){
        std::ifstream infile(inputFile);
        string param;
        string s_value;

        if(!infile.is_open()){
            cout << "Could not open file \"" << inputFile << "\"." << endl;
            return false;
        }

        while(infile >> param >> s_value){
            //cout << param << " = " << s_value << endl;
            if(param == "problem")
                drone_params.problem = s_value;
            else if(param == "size")
                drone_params.size = stoi(s_value);
            else if(param == "creatures")
                drone_params.creatures = stoi(s_value);
            else if(param == "trees")
                drone_params.trees = stoi(s_value);
            else if(param == "targets")
                drone_params.targets = stoi(s_value);
            else if(param == "activation")
                drone_params.activation = stof(s_value);
            else if(param == "maxPhotos")
                drone_params.maxPhotos = stoi(s_value);
            else if(param == "photos")
                drone_params.photos = stoi(s_value);
            else if(param == "discount")
                drone_params.discount = stof(s_value);
            else if(param == "fDiscount")
                drone_params.fDiscount = stof(s_value);
            else if(param == "recognition")
                drone_params.recognition = stof(s_value);
            else if(param == "moving")
                drone_params.moving = stof(s_value);
            else if(param == "entropy")
                drone_params.entropy = stof(s_value);
            else
                cout << "\tWarning: \"" << param << "\" is not a valid parameter." << endl;
        }
        infile.close();
        drone_params.description = "drone[" + std::to_string(drone_params.size)
                                   + ", " + std::to_string(drone_params.creatures)
                                   + ", " + std::to_string(drone_params.trees)
                                   + "]";
        return true;
    }
    
    bool parseMobipickFile(MOBIPICK_PARAMS& mobipick_params, string inputFile){
        std::ifstream infile(inputFile);
        string param;
        string s_value;

        if(!infile.is_open()){
            cout << "Could not open file \"" << inputFile << "\"." << endl;
            return false;
        }
        
        while(infile >> param >> s_value){
            //cout << param << " = " << s_value << endl;
            if(param == "problem")
                mobipick_params.problem = s_value;
            else if(param == "cylinders")
                mobipick_params.cylinders = stoi(s_value);
            else if(param == "objects")
                mobipick_params.objects = stoi(s_value);
            else if(param == "tables")
                mobipick_params.tables = stoi(s_value);
            else if(param == "reqCyls")
                mobipick_params.reqCyls = stoi(s_value);
            else if(param == "discount")
                mobipick_params.discount = stof(s_value);
            else if(param == "fDiscount")
                mobipick_params.fDiscount = stof(s_value);
            else if(param == "entropy")
                mobipick_params.entropy = stof(s_value);
            else if(param == "activation")
                mobipick_params.activation = stof(s_value);
            else if(param == "PGSAlpha")
                mobipick_params.PGSAlpha = stof(s_value);
            else if(param == "transitionRate")
                mobipick_params.transitionRate = stof(s_value);
            else if(param == "identify")
                mobipick_params.identify = stof(s_value);
            else if(param == "perceive")
                mobipick_params.perceive = stof(s_value);
            else if(param == "grasping")
                mobipick_params.grasping = stof(s_value);
            else if(param == "grasping_other")
                mobipick_params.grasping_other = stof(s_value);
            else
                cout << "\tWarning: \"" << param << "\" is not a valid parameter." << endl;
        }
        infile.close();
        
        mobipick_params.description = "mobipick[" + std::to_string(mobipick_params.cylinders)
                                   + ", " + std::to_string(mobipick_params.objects)
                                   + ", " + std::to_string(mobipick_params.tables)
                                   + "]"
                                   + " w/ Req. Cyls = " + std::to_string(mobipick_params.reqCyls);
        return true;
    }
    
};

#endif
