#include "mcts.h"
#include "experiment.h"
#include <iomanip>

// RAGE problem files
#include "rocksample.h"
#include "cellar.h"
#include "drone.h"
#include "mobipick.h"

#include "Parser.h"
#include <fstream>

using namespace std;
//using namespace boost::program_options;

int main(int argc, char* argv[])
{
    //PROBLEM_PARAMS * problem_params;
    MCTS::PARAMS searchParams;
    EXPERIMENT::PARAMS expParams;
    SIMULATOR::KNOWLEDGE knowledge;
    PARSER::COMMAND_LINE cl;

    string problem, inputfile, outputfile, policy;
    int size, number, number2 = 10, number3 = 10;

    PARSER::parseCommandLine(argv, argc, cl);

    problem = cl.problem;
    inputfile = cl.inputFile;
    outputfile = cl.outputFile;
    size = cl.size;
    number = cl.number;
    number2 = cl.number2;
    number3 = cl.number3;

    expParams.TimeOut = cl.timeout;
    expParams.MinDoubles = cl.minDoubles;
    expParams.MaxDoubles = cl.maxDoubles;
    expParams.NumRuns = cl.runs;
    expParams.NumSteps = cl.numSteps;

    searchParams.Verbose = cl.verbose;
    searchParams.useFtable = cl.fTable;

    knowledge.TreeLevel = cl.treeKnowledge;
    knowledge.RolloutLevel = cl.rolloutKnowledge;
    
    if(cl.problem == "none")
    {
        cout << "No problem specified" << endl;
        return 1;
    }

    SIMULATOR* real = 0;
    SIMULATOR* simulator = 0;
    string description;

    if (problem == "rocksample")
    {
        real = new ROCKSAMPLE(size, number);
        simulator = new ROCKSAMPLE(size, number);
        description = "rocksample[" + std::to_string(size) + "," + std::to_string(number) + "]";
    }    
    else if(inputfile == "none"){
            cout << "Problem file required." << endl;
            exit(1);
        }
    else if (problem == "cellar"){
        CELLAR_PARAMS problem_params;
        if(!PARSER::parseCellarFile(problem_params, inputfile)) return 1;
        real = new CELLAR(problem_params);
        simulator = new CELLAR(problem_params);        
        description = problem_params.description;
    }
    else if (problem == "drone"){
        DRONE_PARAMS problem_params;
        if(!PARSER::parseDroneFile(problem_params, inputfile)) return 1;
        real = new DRONE(problem_params);
        simulator = new DRONE(problem_params);
        description = problem_params.description;
    }
    else if (problem == "mobipick"){
        MOBIPICK_PARAMS problem_params;
        if(!PARSER::parseMobipickFile(problem_params, inputfile)) return 1;
        real = new MOBIPICK(problem_params);
        simulator = new MOBIPICK(problem_params);
        description = problem_params.description;
    }
	else{
        cout << "Unknown problem." << endl;
        exit(1);
    }
        
    cout << "Running " << description << endl;

    cout << left << std::setw(8) << "Steps";
    cout << left << std::setw(11) << "Rollouts";
    cout << left << std::setw(6) << "IRE";
    cout << left << std::setw(14) << "Verbosity";
    cout << endl;
    
    cout << left << std::setw(8) << expParams.NumSteps;
    if (knowledge.RolloutLevel == 1) cout << left << std::setw(11) << "Random";
    else if (knowledge.RolloutLevel == 2) cout << left << std::setw(11) << "Smart";
    else if (knowledge.RolloutLevel >= 3) cout << left << std::setw(11) << "PGS";
    
    if (searchParams.useFtable) cout << left << std::setw(6) << "Y";
    else cout << left << std::setw(6) << "N";

    cout << left << std::setw(14) << searchParams.Verbose;
    cout << endl;
	//cout << "Tree level: " << knowledge.TreeLevel << endl;

    simulator->SetKnowledge(knowledge);
    EXPERIMENT experiment(*real, *simulator, outputfile, expParams, searchParams);
    experiment.DiscountedReturn();

    delete real;
    delete simulator;
    return 0;
}
