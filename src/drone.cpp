#include "drone.h"
#include "utils.h"

#include <iomanip>
#include <fstream>

using namespace std;
using namespace UTILS;

void DRONE_STATE::activateFeature(int feature, bool status){
    if(feature >= 0){
        Features[feature].active = status;
    }
}

/* Build Ftable mapping every action to its affected feature/object */
void DRONE::initializeFTable(FTABLE& ftable) const{

//	cout << "Creating FTable for Drone" << endl;

    //Add all 'check room' actions
    /*for(int action=E_CHECK, feature = 0; feature < NumCells; action++, feature++){
        ftable.addEntry(action, feature);
    }*/

    //Feature identify
    for(int action=E_IDENTIFY, feature = 0; feature < NumFeatures; action++, feature++){
       ftable.addEntry(action, feature);
    }

    //Feature photo
    for(int action=E_PHOTO, feature = 0; feature < NumFeatures; action++, feature++){
        ftable.addEntry(action, feature);
    }

    ftable.setACTIVATION_THRESHOLD(DRONE::ACTIVATION_THRESHOLD); ///Problem specific activation threshold
    ftable.setNumActions(NumActions);
    ftable.setNumFeatures(NumFeatures);

//	cout << "Done" << endl;
}

DRONE::DRONE(PROBLEM_PARAMS& problem_params){

    DRONE_PARAMS& params = safe_cast<DRONE_PARAMS&>(problem_params);

    ///Read parameters from struct
    Size = params.size;
    NumCreatures = params.creatures;
    NumTrees = params.trees;
    NumTargets = std::min(params.targets, NumCreatures);

//    cout << Size << ", " << NumCreatures << ", " << NumTrees << ", " << NumTargets << endl;

    Discount = params.discount;  //MCTS discount
    fDiscount = params.fDiscount;  //Feature-value discount.  Probably also problem dependent.

//    cout << Discount << ", " << fDiscount << endl;

    MaxPhotos = params.maxPhotos;
    PhotosNeeded = params.photos;

//    cout << MaxPhotos << ", " << PhotosNeeded << endl;

    ///Object classification accuracy
    RECOGNITION_RATE = params.recognition;
    ///Probability of moving for every object
    PROB_MOVING = params.moving;
    ///Determines when a probability estimate (with Binomial distribution) is 'safe'
    BIN_ENTROPY_LIMIT = params.entropy;
    ///Feature activation threshold
    ACTIVATION_THRESHOLD = params.activation;

    //Setup problem with parameters
    Grid = GRID<int>(Size, Size);
    NumCells = Size*Size;
    NumFeatures = NumCreatures + NumTrees;
    NumActions = 6; // Movement actions + wait + leave
    NumActions += NumCells; //All checks
    NumActions += NumFeatures; //Identify
    NumActions += NumFeatures; //Photos

    //Actions are: N, S, E, W + Wait, Leave
    E_CHECK = 6; //Check, one per cell
    E_IDENTIFY = E_CHECK + NumCells; //First identify
    E_PHOTO = E_IDENTIFY + NumFeatures; //First photo

    //cout << "Num Actions: " << NumActions << endl;

    NumObservations = 4 + NumFeatures; //Check returns ID of creature
    RewardRange = 50; //NOTE: exploration rate recommended to be H_max - H_min, max with c = 0, min with rollouts


    if (Size == 3 && NumCreatures == 3 && NumTrees == 0)
        Init_3_3();
    else
    if (Size == 3 && NumCreatures == 3 && NumTrees == 3)
        Init_3_3_3();
    else
    if(Size == 5 && NumCreatures == 8 && NumTrees == 8)
        Init_5_8_8();
    else
        InitGeneral();
}

DRONE::DRONE(int size, int creatures, int trees, int targets, bool useTable)
        :   Grid(size, size),
            Size(size),
            NumCreatures(creatures),
            NumTrees(trees),
            NumCells(size*size)
            //NumTargets(targets)
{

    NumTargets = std::min(targets, NumCreatures);

    NumFeatures = NumCreatures + NumTrees;
    NumActions = 6; // Movement actions + wait + leave
    NumActions += NumCells; //All checks
    NumActions += NumFeatures; //Identify
    NumActions += NumFeatures; //Photos

    //Actions are: N, S, E, W + Wait, Leave
    E_CHECK = 6; //Check, one per cell
    E_IDENTIFY = E_CHECK + NumCells; //First identify
    E_PHOTO = E_IDENTIFY + NumFeatures; //First photo

    //cout << "Num Actions: " << NumActions << endl;
    useFtable = useTable;

    NumObservations = 4 + NumFeatures; //Check returns ID of creature
    RewardRange = 50; //NOTE: exploration rate recommended to be H_max - H_min, max with c = 0, min with rollouts
    Discount = 0.95;  //MCTS discount
    fDiscount = 0.3;  //Feature-value discount.  Probably also problem dependent.

    MaxPhotos = 1; //TODO: Make MaxPhotos a parameter
    PhotosNeeded = 1; //TODO: Make PhotosNeeded a parameter

    ///Object classification accuracy
    RECOGNITION_RATE = 0.9; //TODO: separate recognition rate from problem definition
    ///Probability of moving for every object
    PROB_MOVING = 0.15; //TODO: separate movement prob from problem definition.  Each entry could have its own pattern too.
    ///Determines when a probability estimate (with Binomial distribution) is 'safe'
    BIN_ENTROPY_LIMIT = 0.4;
    ACTIVATION_THRESHOLD = -16;
    //RandomSeed(0);

    if (size == 3 && creatures == 3 && trees == 0)
        Init_3_3();
    else
    if (size == 3 && creatures == 3 && trees == 3)
        Init_3_3_3();
    else
    if(size == 5 && creatures == 8 && trees == 8)
        Init_5_8_8();
    else
        InitGeneral();
}

void DRONE::Init_3_3(){
    cout << "Using special layout for drone(3, 3)" << endl;

    //Initial creature locations
    COORD creatures[] =
            {
            COORD(0, 2),
            COORD(1, 1),
            COORD(2, 2)
            };

    HalfEfficiencyDistance = 20;
    StartPos = COORD(1, 1);
    Grid.SetAllValues(-1);
    for (int i = 0; i < NumCreatures; ++i)
    {
        Grid(creatures[i]) = i;
        FeaturePos.push_back(creatures[i]);
    }
}

void DRONE::Init_3_3_3(){
    cout << "Using special layout for drone(3, 3, 3)" << endl;

    //Initial creature locations
    COORD creatures[] =
            {
                    COORD(0, 2),
                    COORD(1, 1),
                    COORD(2, 2)
            };
    COORD trees[] =
            {
                    COORD(0, 0),
                    COORD(2, 0),
                    COORD(2, 1)
            };

    HalfEfficiencyDistance = 20;
    StartPos = COORD(1, 1);
    Grid.SetAllValues(-1);
    for (int i = 0; i < NumCreatures; ++i)
    {
        Grid(creatures[i]) = i;
        FeaturePos.push_back(creatures[i]);
    }

    for (int i = 0; i < NumTrees; ++i)
    {
        Grid(trees[i]) = i;
        FeaturePos.push_back(trees[i]);
    }
}

void DRONE::Init_5_8_8(){
    cout << "Using special layout for drone(5, 8, 8)" << endl;

    //Initial creature locations
    COORD creatures[] =
            {
                    COORD(0, 1),
                    COORD(0, 3),
                    COORD(1, 2),
                    COORD(2, 0),
                    COORD(3, 3),
                    COORD(3, 4),
                    COORD(4, 0),
                    COORD(4, 2)
            };
    COORD trees[] =
            {
                    COORD(0, 2),
                    COORD(1, 0),
                    COORD(1, 4),
                    COORD(2, 3),
                    COORD(3, 0),
                    COORD(3, 1),
                    COORD(4, 3),
                    COORD(4, 4)
            };

    HalfEfficiencyDistance = 20;
    StartPos = COORD(2, 4);
    Grid.SetAllValues(-1);
    for (int i = 0; i < NumCreatures; ++i)
    {
        Grid(creatures[i]) = i;
        FeaturePos.push_back(creatures[i]);
    }

    for (int i = 0; i < NumTrees; ++i)
    {
        Grid(trees[i]) = i;
        FeaturePos.push_back(trees[i]);
    }
}

/*
	The Grid exists for consistency across start states.  This way multiple (starting) instances of the same problem (ie. object location)
can be created.  This is used when creating state samples for belief state.
*/
void DRONE::InitGeneral()
{
    HalfEfficiencyDistance = 20;
    StartPos = COORD(Size / 2, Size / 2);
    RandomSeed(0);
    Grid.SetAllValues(-1);
    //Place Features randomly
    for (int i = 0; i < NumFeatures; ++i)
    {
        COORD pos;

        if(i < Size*Size) {
            do {
                pos = COORD(Random(Size), Random(Size));
            } while (Grid(pos) >= 0);
            Grid(pos) = i;
        }
        else{
            pos = COORD(Random(Size), Random(Size));
        }

        FeaturePos.push_back(pos);
    }
}

STATE* DRONE::Copy(const STATE& state) const
{
    const DRONE_STATE& dronestate = safe_cast<const DRONE_STATE&>(state);
    DRONE_STATE* newstate = MemoryPool.Allocate();
    *newstate = dronestate;
    return newstate;
}

void DRONE::Validate(const STATE& state) const
{
    const DRONE_STATE& droneState = safe_cast<const DRONE_STATE&>(state);
    assert(Grid.Inside(droneState.AgentPos));
}

STATE* DRONE::CreateStartState() const
{
    DRONE_STATE* droneState = MemoryPool.Allocate();
    droneState->AgentPos = StartPos;
    droneState->TargetPhotosTaken = 0;
    droneState->NoTargetPhotosTaken = 0;
    droneState->Features.clear();
    //droneState->Rooms.clear();

    //Add bears
    for (int i = 0; i < NumCreatures; i++)
    {
        DRONE_STATE::P_ENTRY entry;

        /** Ground truth & sim **/
        entry.Target = false;
        entry.type = F_BEAR;
        entry.Position = FeaturePos[i];
        /** ** **/

        entry.AssumedTarget = 0;
        entry.ObservedPosition = COORD::Null; //Start with observed positions?
        entry.ProbPosition = 1.0;

        entry.LikelihoodTarget = 1.0;
        entry.LikelihoodNotTarget = 1.0;
        entry.ProbTarget = 0.5;

        entry.numPhotos = 0;
        entry.measured = 0;
        entry.count = 0;
        entry.active = true;

        droneState->Features.push_back(entry);
    }

    //Mark BF targets and change type
    /*
    for(int i=0; i < NumTargets && i < NumFeatures; i++){
        droneState->Features[i].Target = true;
        droneState->Features[i].type = F_BIGFOOT;
    }*/

    //Mark targets randomly
    int target;
    for (int i = 0; i < NumTargets && i < NumCreatures; ++i)
    {
        do
        {
            target = Random(NumCreatures);
        }
        while (droneState->Features[target].Target);
        droneState->Features[target].Target = true;
        droneState->Features[target].type = F_BIGFOOT;
    }

    //Add trees
    for (int i = NumCreatures; i < NumCreatures+NumTrees; i++)
    {
        DRONE_STATE::P_ENTRY entry;

        /** Ground truth & sim **/
        entry.Target = false;
        entry.type = F_TREE;
        entry.Position = FeaturePos[i];
        /** ** **/

        entry.AssumedTarget = 0;
        entry.ObservedPosition = COORD::Null; //Start with observed positions?
        entry.ProbPosition = 1.0;

        entry.LikelihoodTarget = 1.0;
        entry.LikelihoodNotTarget = 1.0;
        entry.ProbTarget = 0.5;

        entry.numPhotos = 0;
        entry.measured = 0;
        entry.count = 0;
        entry.active = true;

        droneState->Features.push_back(entry);
    }

    assert(droneState->Features.size() == NumFeatures);

    return droneState;
}

void DRONE::FreeState(STATE* state) const
{
    DRONE_STATE* droneState = safe_cast<DRONE_STATE*>(state);
    MemoryPool.Free(droneState);
}

bool DRONE::Step(STATE& state, int action,
                  int& observation, double& reward) const
{
    //cout << "Rolloutlevel: " << Knowledge.RolloutLevel << endl;
    if(Knowledge.RolloutLevel >= KNOWLEDGE::PGS){
        //cout << "Calling Step PGS" << endl;
        return StepPGS(state, action, observation, reward);
    }
    else
        return StepNormal(state, action, observation, reward);
}

/*
 * Step function with PGS PBRS
 *
 * Performs regular step and adds the reward bonus
 *      gamma*phi(s') - phi(s)
 * where phi(s) = alpha*PGS(s) and gamma = 1
 *
*/
bool DRONE::StepPGS(STATE& state, int action,
                     int& observation, double& reward) const
{
    double scale = 10.0;
    double r = 0.0;
    double r2 = 0.0;
    STATE* oldstate = Copy(state);

    bool terminal = StepNormal(state, action, observation, reward);

    // Potential-based reward bonus
    if(!terminal){//Not terminal or out of bounds
        r2 = PGS(*oldstate);
        r = PGS_RO(*oldstate, state, action, r2); //PGS(state);
        reward += scale*r - scale*r2;
        /*if(action >= E_IDENTIFY && action < E_PHOTO){
            DisplayAction(action, cout);
            cout << "PGS bonus: " << scale*r - scale*r2 << endl;
        }*/
    }
    FreeState(oldstate);

    return terminal;
}

/*
 * Regular step function.  Simulates transition from state with action and returns observation and reward
*/

bool DRONE::StepNormal(STATE& state, int action,
                        int& observation, double& reward) const
{
    DRONE_STATE& droneState = safe_cast<DRONE_STATE&>(state);
    reward = 0;
    observation = O_NONE;

    //Define reward distribution
    double reward_check = -0.5;
    int reward_move = -1;
    int reward_wait = -1;
    double reward_identify = -0.5;
    int wrong_photo = -10;
    int right_photo = 5;

    int terminal_success = 10;
    int terminal_failure = -100;
    int reward_wrongIdentify = -1; ///Punish any general mistakes (eg. identifying a person in a different room)
    int outOfBounds = -100;
    int deadBattery = -100; //TODO: implement battery consumption

    //if(action > NumActions){
    //    cout << "Starting step function, action no." << action << endl;
    //    DisplayAction(action, cout);
    //     DisplayState(state, cout);
    //}

    ///Setup environmental state transition
    //Move everything (movable) with probability and update their positionProbs
    for(int i=0; i<NumFeatures; i++) {
        if(i < NumCreatures)
            MoveFeature(droneState, i);
        //Update the prob of everything, including trees
        droneState.Features[i].ProbPosition *= (1 - PROB_MOVING);
    }

    if (action < A_LEAVE) // move & wait
    {
        COORD pos;
        reward = reward_move;

        switch (action)
        {
            // A_WAIT is implicitly represented with reward_move but no action
            case A_WAIT:
                reward = reward_wait;
                break;
            case COORD::E_EAST:
                //pos = COORD(droneState.AgentPos.X+1, droneState.AgentPos.Y);
                if (droneState.AgentPos.X + 1 < Size){
                    droneState.AgentPos.X++;
                }
                else{// Maze exit
                    reward = outOfBounds;
                }
                break;

            case COORD::E_NORTH:
                //pos = COORD(cellarstate.AgentPos.X, cellarstate.AgentPos.Y+1);
                if (droneState.AgentPos.Y + 1 < Size){
                    droneState.AgentPos.Y++;
                }
                else
                    reward = outOfBounds;
                break;

            case COORD::E_SOUTH:
                //pos = COORD(cellarstate.AgentPos.X, cellarstate.AgentPos.Y-1);
                if (droneState.AgentPos.Y - 1 >= 0){
                    droneState.AgentPos.Y--;
                }
                else
                    reward = outOfBounds;
                break;

            case COORD::E_WEST:
                //pos = COORD(cellarstate.AgentPos.X-1, cellarstate.AgentPos.Y);
                if (droneState.AgentPos.X - 1 >= 0){
                    droneState.AgentPos.X--;
                }
                else
                    reward = outOfBounds;
                break;
        }
    }

    //Terminal state
    if(action == A_LEAVE){
        if(droneState.TargetPhotosTaken >= PhotosNeeded) {
            reward = terminal_success;
        }
        else {
            reward = terminal_failure;
        }
        return true;
    }

    // Cell/Tile check: Returns observation (Empty, Creature ID) and if positive updates the feature's position
    if (action >= E_CHECK && action < E_IDENTIFY)
    {
        int cell = action - E_CHECK;

        assert(cell < NumCells);
        observation = Observe(droneState, cell); // O_EMPTY, O_FEATURE...O_FEATURE+NumFeatures
        //droneState.Rooms[cell].measured++;

        int featureN = observation - O_FEATURE;

        if (featureN >= 0) //Observed something
        {
            double distance = COORD::EuclideanDistance(droneState.AgentPos, Grid.Coord(cell));
            double efficiency = (1 + pow(2, -distance / HalfEfficiencyDistance)) * 0.5;

            droneState.Features[featureN].ObservedPosition = Grid.Coord(cell);
            droneState.Features[featureN].ProbPosition = efficiency;
        }

        reward += reward_check;
    }

    /* Determine whether a feature is Big Foot or not.  Updates ProbTarget
    */
    if(action >= E_IDENTIFY && action < E_PHOTO){
        //Identify feature
        int feature = action - E_IDENTIFY;

        //cout << "UCB: Attempting to identify feature " << feature << endl;
        ///Attempting to ID needs at least an 'idea' of the feature's location
        if(droneState.Features[feature].ObservedPosition != droneState.AgentPos){
            observation = O_NONE;
            reward += reward_wrongIdentify;
            return false;
        }

        ///But if the feature is not really there, it fails (eg.: system cannot find object in picture)
        if(droneState.Features[feature].Position != droneState.AgentPos){
            observation = O_NONE;
            reward += reward_identify;
            return false;
        }

        ///Identifying a known target has no further effect
        if(droneState.Features[feature].ProbTarget == 0 || droneState.Features[feature].ProbTarget == 1){
            droneState.Features[feature].measured++;
            observation = droneState.Features[feature].Target? O_TARGET : O_NOTARGET;
            reward += reward_identify;
            return false;
        }


        ///Only succeed if the object really is there
        //cout << "UCB: Attempting to identify feature " << feature << endl;
        observation = Identify(droneState, feature);
        droneState.Features[feature].measured++;
        //DisplayObservation(droneState, observation, cout);

        //Normalized Recognition rate (0-1)
        double efficiency = DRONE::RECOGNITION_RATE;

        if (observation == O_TARGET) {
            droneState.Features[feature].count++;
            droneState.Features[feature].LikelihoodTarget *= efficiency;
            droneState.Features[feature].LikelihoodNotTarget *= 1.0 - efficiency;
        } else {
            droneState.Features[feature].count--;
            droneState.Features[feature].LikelihoodNotTarget *= efficiency;
            droneState.Features[feature].LikelihoodTarget *= 1.0 - efficiency;
        }

        //Update target probability
        double denom = (0.5 * droneState.Features[feature].LikelihoodTarget) +
                       (0.5 * droneState.Features[feature].LikelihoodNotTarget);
        droneState.Features[feature].ProbTarget = (0.5 * droneState.Features[feature].LikelihoodTarget) / denom;

        //If entropy is reduced, target status may be assumed
        if(!droneState.Features[feature].AssumedTarget) {
            if (BinEntropyCheck(droneState.Features[feature].ProbTarget)) {
                if (round(droneState.Features[feature].ProbTarget))
                    droneState.Features[feature].AssumedTarget = true;
                else
                    droneState.Features[feature].AssumedTarget = false;
            }
        }
        reward += reward_identify;
    }

    //Take photo of feature when standing directly above
    //Should reward if belief that target is in room is high, unless photo reveals target
    if (action >= E_PHOTO){
        droneState.PhotosTaken++;

        ///Limit number of pictures?
        /*if(droneState.PhotosTaken > MaxPhotos){
            observation = O_NONE;
            reward = wrong_photo;
            return false;
        }*/

        ///Succeed only if taking a picture directly above a real target
        int feature = action - E_PHOTO;
        if(FeatureAt(droneState, feature, droneState.AgentPos)){
            droneState.Features[feature].numPhotos++; //Photo successful

            ///Option 1: reward photos of assumed targets.  Problematic if info is wrong
            /*if(droneState.Features[feature].AssumedTarget){
                observation = O_TARGET;
                droneState.TargetPhotosTaken++;
                reward += right_photo / droneState.TargetPhotosTaken;
            }
            else{
                observation = O_NOTARGET;
                droneState.NoTargetPhotosTaken++;
                reward += wrong_photo;
            }*/

            ///Option 2: Reward only real targets.  Must reveal their type for consistency
            if(droneState.Features[feature].Target){
                droneState.Features[feature].ProbTarget = 1.0;
                droneState.Features[feature].AssumedTarget = true;
                observation = O_TARGET;
                droneState.TargetPhotosTaken++;
                reward += right_photo / droneState.TargetPhotosTaken;
            }
            else{
                droneState.Features[feature].ProbTarget = 0.0;
                droneState.Features[feature].AssumedTarget = false;
                observation = O_NOTARGET;
                droneState.NoTargetPhotosTaken++;
                reward += wrong_photo;
            }
        }
        else
        {
            observation = O_NONE;
            //droneState.NoTargetPhotosTaken++;
            reward += wrong_photo; //TODO: also scale these rewards?
        }
    }

    //cout << "Step done. " << endl;
    //DisplayObservation(droneState, observation, cout);

    return false;
}

void DRONE::MoveFeature(DRONE_STATE &droneState, int feature) const {
    int x,y;
    int p_move = DRONE::PROB_MOVING*100; //0-100
    //for(std::vector<DRONE_STATE::P_ENTRY>::iterator it = droneState.Features.begin(); it != droneState.Features.end(); ++it){
    //for(int i=0; i < NumCreatures; i++){
    x = 0;
    y = 0;
    if(Random(100) < p_move) {
        switch (Random(4)) {
            case COORD::E_NORTH:
                y = 1;
                break;
            case COORD::E_SOUTH:
                y = -1;
                break;
            case COORD::E_EAST:
                x = 1;
                break;
            case COORD::E_WEST:
                x = -1;
                break;
        }
        int newX = droneState.Features[feature].Position.X + x;
        int newY = droneState.Features[feature].Position.Y + y;

        if (newX >= 0 && newX < Size && newY >= 0 && newY < Size && EmptyCell(droneState, COORD(newX, newY))) {
            droneState.Features[feature].Position.X = newX;
            droneState.Features[feature].Position.Y = newY;
        }
    }

    //}
}


// Drone domain transformations -- change a random target
bool DRONE::LocalMove(STATE& state, const HISTORY& history,
                       int stepObs, const STATUS& status) const
{
    DRONE_STATE& droneState = safe_cast<DRONE_STATE&>(state);
    int feature;

    //Modify feature
    feature = Random(NumFeatures);

    //Change its type
    droneState.Features[feature].Target = !(droneState.Features[feature].Target);

    //Validate all previous photos and ID's:
    //Changing target status of known creatures is not a valid belief
    if( (droneState.Features[feature].ProbTarget == 0 && droneState.Features[feature].Target) ||
        (droneState.Features[feature].ProbTarget == 1 && !droneState.Features[feature].Target))
        return false;

    //And move if possible
    MoveFeature(droneState, feature);

    //cout << "Local move: " << endl;
    //DisplayState(droneState, cout);

    //Validate
    int action = history.Back().Action;

    //CHECK: validate location observations
    if(action >= E_CHECK && action < E_IDENTIFY){
        int cell = action - E_CHECK;
        int realObs = history.Back().Observation;
        int newObs = Observe(droneState, cell);

        //Check the same cell and see if observations match
        if(newObs != realObs)
            return false;
    }

    //IDENTIFY: validate id observations
    if (action >= E_IDENTIFY && action < E_PHOTO) {
        //cout << "Local move with action: ";
        //DisplayAction(action, cout);

        feature = action - E_IDENTIFY;
        int realObs = history.Back().Observation;
        int newObs = Identify(droneState, feature);

        //ID the same feature and see if observations match
        if (newObs != realObs)
            return false;
    }

    //PHOTO: validate photo observations.  If a photo revealed a target (or not target) its type cannot be changed
    //Use only if photos reveal target status 100%
    if (action >= E_PHOTO) {
        feature = action - E_PHOTO;
        int realObs = history.Back().Observation; //Target, No Target, NONE

        // Condition new state on real observation
        int newObs = droneState.Features[feature].Target ? O_TARGET : O_NOTARGET;

        if (newObs != realObs)
            return false;
    }

    return true;
}

/* Fast PGS for Rollout policy
 * Simplified PGS point count by using only the specific action changes
 */
double DRONE::PGS_RO(STATE& oldstate, STATE& state, int action, double oldpgs) const
{
    double points = 0.0;
    double oldpoints = 0.0;

    //1. Cast to cellarstate
    DRONE_STATE& droneState = safe_cast<DRONE_STATE&>(state);
    DRONE_STATE& oldDroneState = safe_cast<DRONE_STATE&>(oldstate);

    int creature;
    int cell;
    //1. Photos
    if(action >= E_PHOTO){
        creature = action - E_PHOTO;
        if (droneState.Features[creature].Position == droneState.AgentPos){
            if(droneState.Features[creature].Target){
                if(droneState.Features[creature].AssumedTarget &&
                    !oldDroneState.Features[creature].numPhotos)
                points++; //Add one point for the first (correct) picture
            }
            else points--;
        }
    }
    //2. Identify
    else if (action >= E_IDENTIFY){
        creature = action - E_IDENTIFY;
        double p = droneState.Features[creature].ProbTarget;
        if(!BinEntropyCheck(p)) points--;

        p = oldDroneState.Features[creature].ProbTarget;
        if(!BinEntropyCheck(p)) oldpoints--;
    }
    //3. Location of creatures
    /*else if (action >= E_CHECK){
        //Award point if check revealed something
        cell = action- E_CHECK;
        creature = FeatureInCell(droneState, cell);
        for(int i=0; i < NumCreatures; i++) {
            if (droneState.Features[i].ObservedPosition == Grid.Coord(cell))
                points--;
            if (oldDroneState.Features[i].ObservedPosition == COORD::Null)
                oldpoints--;
        }
    }*/

    //Update difference for current action
    double result = oldpgs - oldpoints + points;

    return result;
}

/*
 * PGS point count
 */
double DRONE::PGS(STATE& state) const
{
    double points = 0.0;
    double prob_position = 0.3;

    //1. Cast
    DRONE_STATE& droneState = safe_cast<DRONE_STATE&>(state);

    //2. Points for taking pictures
 //   points += droneState.TargetPhotosTaken;
 //   points -= droneState.NoTargetPhotosTaken;

    double p_target, binaryEntropy;
    for(int feature=0; feature < NumFeatures; feature++){
        //bool positionSafe = droneState.Features[feature].ProbPosition >= prob_position;

        //2. Award points for taking pictures of targets with good observations
        if (droneState.Features[feature].numPhotos){
            if(droneState.Features[feature].Target){
                if(droneState.Features[feature].AssumedTarget)
                    points++; // += droneState.Features[feature].numPhotos;
            }
            else
                points--; // -= droneState.Features[feature].numPhotos;
        }

        //3. Negative points for unidentified features
        p_target = droneState.Features[feature].ProbTarget;
        if(!BinEntropyCheck(p_target)) points--;


        //4. Points for tracking promising, active features?
        //if(BinEntropyCheck(p_target) && p_target > 0.5 && positionSafe)
        //if(droneState.Features[feature].active &&
        //    positionSafe &&
        //   p_target > 0.5)
        //    points++;

        //5.  Deduct points for missing features
        //if(droneState.Features[feature].ObservedPosition == COORD::Null)
        //    points--;
    }

    return points;
}

// PGS Rollout policy
// Computes PGS only for non Checking actions
//TODO: consider going back to "fast" RO PGS
void DRONE::GeneratePGS(const STATE& state, const HISTORY& history,
                         vector<int>& legal, const STATUS& status) const
{
    static vector<int> acts;
    acts.clear();
    STATE * newstate;
    PGSLegal(state, history, acts, status);
    int numLegal = acts.size();

    double pgs_values[numLegal];

    STATE * oldstate = Copy(state);
    double pgs_state = PGS(*oldstate);

    int max_p = -1;
    double max_v = -Infinity;

    int observation;
    double reward;

    //cout << "Generating PGS values..." << endl;
    //cout << "Found " << numLegal << " legal actions." << endl;

    /*
     * Simulate a transition for each action (eg. lookahead) to get its PGS value
     * */
    for(unsigned int i=0; i<numLegal; i++){
        newstate = Copy(state);
        StepNormal(*newstate, acts[i], observation, reward); //Simulate transition with action a
        pgs_values[i] = PGS_RO(*oldstate, *newstate, acts[i], pgs_state);//Add only PGS differences (fast) //PGS(*newstate);
        FreeState(newstate);
    }

    FreeState(oldstate);
    max_p = std::distance(pgs_values, max_element(pgs_values, pgs_values+numLegal));
    max_v = pgs_values[max_p];
    assert(max_p > -1);

    /*
     * Return action with highest PGS, break ties randomly
     * */
    legal.push_back(acts[max_p]); //Add best action to return vector
    // Add other maxima
    for(int i=0; i<numLegal; i++){
        if(i != max_p && pgs_values[i] == max_v)
            //if(pgs_values[i] >= 0.5)
            legal.push_back(acts[i]);
    }

    //cout << "found " << legal.size() << " rollout actions " << endl;
}

/*
 * Legal actions following PGS policy: avoid actions that do not reduce uncertainty
*/

void DRONE::PGSLegal(const STATE& state, const HISTORY& history,
                      vector<int>& legal, const STATUS& status) const
{
    //GenerateLegal(state, history, legal, status);

    const DRONE_STATE& droneState =
            safe_cast<const DRONE_STATE&>(state);

    //Move is always possible
    if (droneState.AgentPos.Y + 1 < Size)
        legal.push_back(COORD::E_NORTH);

    if(droneState.AgentPos.X + 1 < Size)
        legal.push_back(COORD::E_EAST);

    if (droneState.AgentPos.Y - 1 >= 0)
        legal.push_back(COORD::E_SOUTH);

    if (droneState.AgentPos.X - 1 >= 0)
        legal.push_back(COORD::E_WEST);

    legal.push_back(A_WAIT);
    legal.push_back(A_LEAVE);

    for(int i=0; i<NumCells; i++) {
        legal.push_back(E_CHECK + i);
    }

    //Use uncertainty reducing tips to:
    //1. Take pictures if a cell might contain the target
    //2. Identify a feature if it is not well known and its location is likely
    for(int f=0; f<NumFeatures; f++) {
//        cout << "RO: Identify f " << people[f] << " in room " << Grid.Index(droneState.AgentPos) << endl;
        if(droneState.Features[f].ObservedPosition == droneState.AgentPos &&
            //droneState.Features[f].ProbPosition >= 0.5 &&
            droneState.Features[f].active) {
                legal.push_back(E_PHOTO + f);
//                if(!droneState.Features[f].AssumedTarget)
                    legal.push_back(E_IDENTIFY + f);
        }
    }

}


void DRONE::GenerateLegal(const STATE& state, const HISTORY& history,
                           vector<int>& legal, const STATUS& status) const
{

    const DRONE_STATE& droneState =
            safe_cast<const DRONE_STATE&>(state);

    //Move is always possible
    if (droneState.AgentPos.Y + 1 < Size)
        legal.push_back(COORD::E_NORTH);

    if(droneState.AgentPos.X + 1 < Size)
        legal.push_back(COORD::E_EAST);

    if (droneState.AgentPos.Y - 1 >= 0)
        legal.push_back(COORD::E_SOUTH);

    if (droneState.AgentPos.X - 1 >= 0)
        legal.push_back(COORD::E_WEST);

    legal.push_back(A_WAIT);
    legal.push_back(A_LEAVE);

    //Check any room
    for(int i=0; i<NumCells; i++)
        legal.push_back(E_CHECK + i);

    //Identify features believed to be in current room
    for(int f=0; f<NumFeatures; f++) {
//        cout << "RO: Identify f " << people[f] << " in room " << Grid.Index(droneState.AgentPos) << endl;
        if(droneState.Features[f].ObservedPosition == droneState.AgentPos) {
            legal.push_back(E_IDENTIFY + f);
            legal.push_back(E_PHOTO + f);
        }
    }

}

/*
  Preferred actions RO policy
  NOTE: This incomplete and most likely useless.
*/
void DRONE::GeneratePreferred(const STATE& state, const HISTORY& history,
                               vector<int>& actions, const STATUS& status) const
{
    const DRONE_STATE& droneState =
            safe_cast<const DRONE_STATE&>(state);

    COORD pose = droneState.AgentPos;

    //Process the people in the room
    int person;
    int room = Grid.Index(pose);
    bool roomPhoto = false;
    bool identify = false;

    for(int i=0; i<NumCreatures; i++){
        person = i;

        // Identify people if their target status is not well known
        // and its location IS the current room
        if(droneState.Features[person].measured < 3 &&
           std::abs(droneState.Features[person].count) < 2 &&
           droneState.Features[person].ProbTarget != 1.0 &&
           droneState.Features[person].ProbTarget != 0.0 &&
           droneState.Features[person].ObservedPosition == pose){

            identify = true;
            actions.push_back(E_IDENTIFY + person);
        }

        //Photograph the room if this person appears to be the target
        if(droneState.Features[person].ProbTarget > 0.7 &&
           std::abs(droneState.Features[person].count) > 2 &&
           droneState.Features[person].measured > 2 &&
           droneState.Features[person].ObservedPosition == pose &&
           !roomPhoto) {
            roomPhoto = true; //Add action only once
            actions.push_back(E_PHOTO + room);
        }
    }

    //Move in the direction of potential targets only if no picture can be taken

    //If interesting people are spotted, follow them
    bool moveN = false;
    bool moveS = false;
    bool moveE = false;
    bool moveW = false;
    bool stay = false;
    for(int i=0; i<NumCreatures && !roomPhoto && !identify; i++){
        if(droneState.Features[i].ObservedPosition != COORD::Null && droneState.Features[i].Target >= 0.5){
            //determine relative direction
            if(droneState.Features[i].ObservedPosition.Y > pose.Y) //North
               moveN = true;
            if(droneState.Features[i].ObservedPosition.Y < pose.Y) //South
               moveS = true;
            if(droneState.Features[i].ObservedPosition.X > pose.X) //East
               moveE = true;
            if(droneState.Features[i].ObservedPosition.X < pose.X) //West
               moveW = true;
            if(droneState.Features[i].ObservedPosition == pose) //Wait
               stay = true;
        }
    }

    if(moveN) actions.push_back(COORD::E_NORTH);
    if(moveS) actions.push_back(COORD::E_SOUTH);
    if(moveE) actions.push_back(COORD::E_EAST);
    if(moveW) actions.push_back(COORD::E_WEST);
    if(stay) actions.push_back(A_WAIT);

    if(droneState.TargetPhotosTaken >= PhotosNeeded)
        actions.push_back(A_LEAVE);

    //Check the rooms adjacent to the last known location of potential targets
    //for(int i=0; i<NumCells && !identify && !roomPhoto; i++) {

    int posX, posY;
    for(int j=0; j<NumCreatures; j++){
        if(droneState.Features[j].ObservedPosition != COORD::Null &&
           droneState.Features[j].ProbTarget >= 0.5) {
            //int roomN = droneState.Features[j].ObservedPosition.Y+1
            posY = droneState.Features[j].ObservedPosition.Y;
            posX = droneState.Features[j].ObservedPosition.X;
            if(posY < Size-1) //North
                actions.push_back(E_CHECK + Grid.Index(posX, posY+1));
            if(posY > 0) //South
                actions.push_back(E_CHECK + Grid.Index(posX, posY-1));
            if(posX < Size-1) //East
                actions.push_back(E_CHECK + Grid.Index(posX+1, posY));
            if(posX > 0) //West
                actions.push_back(E_CHECK + Grid.Index(posX-1, posY));
//            actions.push_back(E_CHECK + room);
        }
    }


        //}
    //}

}

///// Domain specific //////
/*
 * Check whether a feature is in fact at the specified location
 * */
bool DRONE::FeatureAt(const DRONE_STATE &droneState, int feature, const COORD &cell) const {
    assert(feature >= 0 && feature < NumFeatures);
    return droneState.Features[feature].Position == cell;
}

/*
 * Find out whether a room contains at least one target
 * */
bool DRONE::TargetIn(const DRONE_STATE &droneState, int cell) const {
    COORD coord = Grid.Coord(cell);
    bool found = false;

    for(int i=0; i < droneState.Features.size() && !found; i++) {
        if (droneState.Features[i].Position == coord && droneState.Features[i].Target){ //Person in room
            found = true;
        }
    }
    return found;
}

/*
 * Feature ID currently in cell, or empty
 * */
int DRONE::FeatureInCell(const DRONE_STATE &droneState, int cell) const {
    COORD coord = Grid.Coord(cell);
    return FeatureInCoord(droneState, coord);
}

int DRONE::FeatureInCoord(const DRONE_STATE &droneState, const COORD& coord) const {
    int feature = O_EMPTY;

    for (int i = 0; i < droneState.Features.size(); i++){
        if (droneState.Features[i].Position == coord)
            feature = O_FEATURE + i;
    }

    return feature;
}

void DRONE::PeopleInCurrentRoom(const DRONE_STATE &droneState, std::vector<int>& people) const {
    people.clear();

    COORD coord = droneState.AgentPos;

    for (int i = 0; i < droneState.Features.size(); i++){
        if (droneState.Features[i].Position == coord)
            people.push_back(i);
    }
}

//TODO: find alternative to avoid checking all objects for position
bool DRONE::EmptyCell(const DRONE_STATE &droneState, const COORD &coord) const {
    bool empty = true;

    // If there is anything, not empty
    for (int i = 0; i < droneState.Features.size() && empty; i++){
        if (droneState.Features[i].Position == coord)
            empty = false;
    }

    return empty;
}

int DRONE::NumPeopleInRoom(const DRONE_STATE &droneState, const COORD &coord) const {
    int nPeople = 0;
    for (int i = 0; i < droneState.Features.size(); i++){
        if (droneState.Features[i].Position == coord)
            nPeople++;
    }
    return nPeople;
}

/////
/*
 * Identify person observations:
 *
 * With p = recognition_rate, identify whether person is indeed target or not.
 * If fails, person is misclassified.
 * */
int DRONE::Identify(const DRONE_STATE &droneState, int feature) const {
    int obs;

    if (Bernoulli(DRONE::RECOGNITION_RATE)){ //Pass with noisy sensor
        obs = droneState.Features[feature].Target ? O_TARGET : O_NOTARGET;
    }
    else{
        obs = droneState.Features[feature].Target ? O_NOTARGET : O_TARGET;
    }

    return obs;
}


/*
 * Cell check observations:
 *  With prob. p gets a correct reading
 *
 *  With prob 1-p gets a bad reading, i.e. empty U {feature} with uniform prob.
 *
 * */
int DRONE::Observe(const DRONE_STATE &droneState, int cell) const
{
    double distance;
    double efficiency;
    int obs;
    COORD coord = Grid.Coord(cell);

    distance = COORD::EuclideanDistance(droneState.AgentPos, coord);
    efficiency = (1 + pow(2, -distance / HalfEfficiencyDistance)) * 0.5;
    int trueObs = FeatureInCell(droneState, cell);

    if(Bernoulli(efficiency)){ //Correct reading
        obs = trueObs;
    }
    else{ //Incorrect reading
        obs = O_FEATURE + Random(0, NumFeatures);

//        if(obs == NumFeatures)
//           obs = O_EMPTY;
    }

    return obs;
}

/*
 * Return whether value p satisfies the entropy restriction < 0.5 in a Bernoulli distribution
 * */
bool DRONE::BinEntropyCheck(double p) const {
    double binaryEntropy = -1*p*log2(p) - (1-p)*log2(1-p);
    return (binaryEntropy <= DRONE::BIN_ENTROPY_LIMIT);
}

void DRONE::DisplayBeliefs(const BELIEF_STATE& beliefState,
                            std::ostream& ostr) const
{
    int numSamples = beliefState.GetNumSamples();

    ostr << numSamples << " belief samples: ";
    for (int i=0; i<numSamples; i++)
    {
        const STATE* s = beliefState.GetSample(i);
        cout << "Sample " << i+1 << ":" << endl;
        DisplayState(*s, cout);
    }
    ostr << endl;
}

void DRONE::DisplayState(const STATE& state, std::ostream& ostr) const
{
    const DRONE_STATE& droneState = safe_cast<const DRONE_STATE&>(state);
    ostr << endl;
    for (int x = 0; x < Size + 2; x++)
        ostr << "# ";
    ostr << endl;

    int numP;

    for (int y = Size - 1; y >= 0; y--)
    {
        ostr << "# ";
        for (int x = 0; x < Size; x++) {
            COORD pos(x, y);

            if (droneState.AgentPos == COORD(x, y))
                ostr << "* ";
            else{
                numP = FeatureInCoord(droneState, pos);
                if(numP >= O_FEATURE){
                    numP -= O_FEATURE;

                    switch(droneState.Features[numP].type){
                        case F_BIGFOOT:
                            ostr << "X";
                            break;
                        case F_BEAR:
                            ostr << "m";
                            break;
                        case F_TREE:
                            ostr << "T";
                            break;
                    }
                    if(droneState.Features[numP].Target) ostr << "!";
                    else ostr << " ";

                }
                else{
                    ostr << "  ";
                }

            }
        }
        ostr << "#" << endl;
    }
    for (int x = 0; x < Size + 2; x++)
        ostr << "# ";

    ostr << "\nAgent is at: ("<< droneState.AgentPos.X << ", " << droneState.AgentPos.Y << ")" << endl;
    ostr << "Good photos: " << droneState.TargetPhotosTaken << " / " << droneState.PhotosTaken << endl;
    //ostr << "\nFeatures: " << endl;
    ostr << "Real state:\t\t|Belief sample:" << endl;
    ostr << "No.\tType\tR.Pos\t|O.Pos\tP(pos)\tP(BF)\tAssumption" << endl;
    for(int i=0; i<droneState.Features.size(); i++){
        ostr << i << "\t";
        switch(droneState.Features[i].type){
            case F_BIGFOOT:
                ostr << "BigF";
                break;
            case F_BEAR:
                ostr << "Bear";
                break;
            case F_TREE:
                ostr << "Tree";
                break;
            default:
                ostr << "N/A";
                break;
        }
        ostr << "\t";
        ostr << "(" << droneState.Features[i].Position.X << ", " << droneState.Features[i].Position.Y << ")" << "\t";
        ostr << "|(";
        if(droneState.Features[i].ObservedPosition.X >= 0)
            ostr << droneState.Features[i].ObservedPosition.X;
        else
            ostr << "?";
        ostr << ", ";
        if(droneState.Features[i].ObservedPosition.Y >= 0)
            ostr << droneState.Features[i].ObservedPosition.Y;
        else
            ostr << "?";
        ostr << ")" << "\t"
            << std::setprecision(4) << droneState.Features[i].ProbPosition << "\t"
            << std::setprecision(4) << droneState.Features[i].ProbTarget << "\t";
        if(droneState.Features[i].AssumedTarget)
            ostr << "Y";
        else
            ostr << "N";
        ostr << endl;
    }
    ostr << endl;
}

void DRONE::DisplayObservation(const STATE& state, int observation, std::ostream& ostr) const
{
    ostr << "Observed ";
    switch (observation)
    {
        case O_EMPTY:
            ostr << "a patch of ground." << endl;
            break;
        case O_NONE:
            ostr << "nothing." << endl;
            break;
        case O_TARGET:
            ostr << "Big Foot!" << endl;
            break;
        case O_NOTARGET:
            ostr << "something, but not Big Foot." << endl;
            break;
    }

    if(observation >= O_FEATURE) {
        int n = observation - O_FEATURE;
        ostr << "feature " << n << endl;
    }
}

void DRONE::DisplayAction(int action, std::ostream& ostr) const
{
    ostr << "A: ";
    if (action < A_WAIT)
        ostr << "Move " << COORD::CompassString[action] << endl;
    else if(action == A_WAIT)
        ostr << "Wait" << endl;
    else if(action == A_LEAVE)
        ostr << "Leave" << endl;
    else if(action >= E_CHECK && action < E_IDENTIFY)
        ostr << "Check coord (" << Grid.Coord(action - E_CHECK).X << ", " << Grid.Coord(action - E_CHECK).Y << ")" << endl;
    else if(action >= E_IDENTIFY && action < E_PHOTO)
        ostr << "Identify feature " << action - E_IDENTIFY << endl;
    else if(action >= E_PHOTO)
        ostr << "Photograph feature " << action - E_PHOTO << endl;

}

