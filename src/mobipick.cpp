#include "mobipick.h"
#include "utils.h"

#include <iomanip>
#include <fstream>

using namespace std;
using namespace UTILS;

void MOBIPICK_STATE::activateFeature(int feature, bool status){
    // Find object id within Tables
    bool done = false;
    if(feature >= 0){
        for(auto& t : Tables){
            for(auto& o : t.Objects){
                if(o.id == feature){
                    o.active = status;
                    done = true;
                }
                if(done) break;
            }
            if(done) break;
        }
    }
}

MOBIPICK_STATE::~MOBIPICK_STATE(){
    for(auto& t : Tables){
        t.Objects.clear();
    }
    Tables.clear();
}

/* Build Ftable mapping every action to its affected feature/object */
/*
    In Mobipick, actions afforded by objects are: pick object, identify object
 */
void MOBIPICK::initializeFTable(FTABLE& ftable) const{
// Add all corresponding action-feature pairs:
//    ftable.addEntry(action, feature);
    
    //Add all picks
    for(int f = 0; f < NumFeatures; f++)
        ftable.addEntry(A_PICK + f, f);
    
    //Add all identifys
    for(int f = 0; f < NumFeatures; f++)
        ftable.addEntry(A_IDENTIFY + f, f);

    ftable.setACTIVATION_THRESHOLD(MOBIPICK::ACTIVATION_THRESHOLD); ///Problem specific activation threshold
    ftable.setNumActions(NumActions);
    ftable.setNumFeatures(NumFeatures);

//	cout << "Done" << endl;
}

MOBIPICK::MOBIPICK(PROBLEM_PARAMS& problem_params){

    MOBIPICK_PARAMS& params = safe_cast<MOBIPICK_PARAMS&>(problem_params);

    ///Read parameters from struct
    NumCylinders = params.cylinders;
    NumObjects = params.objects;
    NumTables = params.tables;
    ReqCyls = std::min(params.reqCyls, NumCylinders);

    Discount = params.discount;  //MCTS discount
    fDiscount = params.fDiscount;  //Feature-value discount.  Probably also problem dependent.

//    cout << Discount << ", " << fDiscount << endl;
        
    NumPoses = 3 + NumTables*2; //TODO: Have specific poses for each object?
    NumFeatures = NumObjects;

    ///Object identification accuracy
    IDENTIFY_ACC = params.identify;
    ///Table perception accuracy
    PERCEIVE_ACC = params.perceive;
    ///Grasping success probability
    PROB_GRASP = params.grasping;
    PROB_GRASP_OTHER = params.grasping_other;
    ///Probability of moving for every object
    PROB_MOVING = params.moving;
    ///Determines when a probability estimate (with Binomial distribution) is 'safe'
    BIN_ENTROPY_LIMIT = params.entropy;
    ///Feature activation threshold
    ACTIVATION_THRESHOLD = params.activation;
    ///PGS Alpha scaling factor
    PGSAlpha = params.PGSAlpha;
    
    IDENTIFY_THRESHOLD = BIN_ENTROPY_LIMIT; //TODO: Set separate parameter

    //SIMULATOR CLASS PARAMETERS
    NumActions = NumPoses + 2*NumObjects + 1 + 1; //Navigate to every pose, pick+id every object, place at location, perceive at location
    NumObservations = 1 + O_TABLE + NumTables;
    
    RewardRange = 30; //NOTE: exploration rate recommended to be H_max - H_min, max with c = 0, min with rollouts
    
    //Set action markers
    A_NAVIGATION = 0;
    A_PICK = A_NAVIGATION + NumPoses;
    A_IDENTIFY = A_PICK + NumObjects;
    A_PLACE = A_IDENTIFY + NumObjects;
    A_PERCEIVE = A_PLACE + 1;
    
    //Set pose markers
    P_TABLE = 3;
    P_NEAR = P_TABLE + NumTables;
    
    if (NumCylinders == 3 && NumObjects == 6 && NumTables == 2)
        Init_Demo1();
    else
        InitGeneral();
}

//Demo1: 3 cylinders, 6 objects, 2 tables
void MOBIPICK::Init_Demo1(){
    cout << "Using Mobipick Demo 1" << endl;
        
    initTables.clear();

    vector<int> table1{F_NOCYL, F_CYL, F_NOCYL};
    vector<int> table2{F_CYL, F_NOCYL, F_CYL};
    
    //Setup desired configuration but let CreateStartState do the actual assignment
    initTables.push_back(table1);
    initTables.push_back(table2);
    
}

//General problem setup: n cyls, m objects, k tables
void MOBIPICK::InitGeneral()
{
    HalfEfficiencyDistance = 20;    
    RandomSeed(0);
    
    //Place Objects randomly
    initTables.resize(NumTables);
    
    int numO = NumObjects / NumTables; //Assign ratio of cyls to objects per table randomly
    
    int extra = NumObjects % NumTables;
    
    for (auto& table : initTables){
        
        table.resize(numO + extra);
        
        for(auto& o : table){
            o = F_NOCYL; //assume no cyl
        }
        
        extra = 0;
    }
        
    int countC = 0;
    
    //Convert some objs to cyls
    while(countC < NumCylinders){
        int tab = Random(NumTables);
        int obj = Random(initTables[tab].size());        
        
        if(initTables[tab][obj] == F_NOCYL){
            initTables[tab][obj] = F_CYL; //add one cylinder
            countC++; //increase count
        }
    }
}

STATE* MOBIPICK::Copy(const STATE& state) const
{
    const MOBIPICK_STATE& mobipickState = safe_cast<const MOBIPICK_STATE&>(state);
    MOBIPICK_STATE* newstate = MemoryPool.Allocate();
    *newstate = mobipickState;
    return newstate;
}

void MOBIPICK::Validate(const STATE& state) const
{
    const MOBIPICK_STATE& mobipickState = safe_cast<const MOBIPICK_STATE&>(state);
    assert(mobipickState.AgentPose >= 0 && mobipickState.AgentPose < NumPoses);
}

STATE* MOBIPICK::CreateStartState() const
{
    
    MOBIPICK_STATE* mobipickState = MemoryPool.Allocate();
    mobipickState->AgentPose = P_OTHER;
    
    //Clear Tables array
    mobipickState->Tables.clear();

    //Add tables and objects
    int id = 0;
    int o_id = 0;
    for (auto table : initTables){
        MOBIPICK_STATE::TABLE t;
        
        t.id = id++;

        //Add objects
        for(auto o_type : table){
            MOBIPICK_STATE::OBJECT o;
            
            //Ground truth:
            o.id = o_id++;
            o.type = o_type; //F_CYL, F_NOCYL
        
            //Probabilistic properties:
            o.measured = 0;
            o.count = 0;
            
            o.ProbCyl = 0.5;            
            o.LikelihoodCyl = 1.0;
            o.LikelihoodNotCyl = 1.0;
            
            o.PosKnown = false;
            o.ProbPos = 0.5;
            o.LikelihoodPos = 1.0;
            o.LikelihoodNotPos = 1.0;
            
            o.active = true;
            
            //Add to object array
            t.Objects.push_back(o);
        }
        
        //Add table to array
        mobipickState->Tables.push_back(t);
    }

    //Set object in grasp
    //mobipickState->inGrasp = NULL;
    mobipickState->grasping = false;
    
    //Set basket
    mobipickState->Basket.pose = P_BASKET;

    return mobipickState;
}

void MOBIPICK::FreeState(STATE* state) const
{
    MOBIPICK_STATE* mobipickState = safe_cast<MOBIPICK_STATE*>(state);
    MemoryPool.Free(mobipickState);
}

bool MOBIPICK::Step(STATE& state, int action,
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
bool MOBIPICK::StepPGS(STATE& state, int action,
                     int& observation, double& reward) const
{
    double scale = 10.0;
    double r = 0.0;
    double r2 = 0.0;
    STATE* oldstate = Copy(state);

    bool terminal = StepNormal(state, action, observation, reward);

    // Potential-based reward bonus
    //if(!terminal){//Not terminal or out of bounds
        r2 = PGS(*oldstate);
        r = PGS_RO(*oldstate, state, action, r2); //PGS(state);
        
        reward += PGSAlpha*r - PGSAlpha*r2;        
    //}
    FreeState(oldstate);

    return terminal;
}

/*
 * Regular step function.  Simulates transition from state with action and returns observation and reward
*/

bool MOBIPICK::StepNormal(STATE& state, int action,
                        int& observation, double& reward) const
{
    MOBIPICK_STATE& mobipickState = safe_cast<MOBIPICK_STATE&>(state);
    reward = 0;
    observation = O_NONE;

    //Reward distribution    
    double reward_perceive = -0.5;
    double reward_identify = -0.5;
    int reward_move = -1;
    
    int reward_grasp = -1;
    int reward_grasp_fail = -2;
    int reward_bad_grasp = -5;
    int reward_place = -1;
    int reward_bad_place = -5;
    
    int reward_bad = -10;
    int reward_good = 10;

    int terminal_success = 0;
    int terminal_fail = -10;
        
    //if(action > NumActions){
    //    cout << "Starting step function, action no." << action << endl;
    //    DisplayAction(action, cout);
    //     DisplayState(state, cout);
    //}

    ///Setup environmental state transition
    //Move everything that can move
    
    ///Process action and simulate results
    /*
     
    Actions are a sequence of numbers dictated by the following ranges:
    
    A_NAVIGATION = 0 : NumPoses
    A_PICK = A_NAVIGATION + NumPoses : A_IDENTIFY
    A_IDENTIFY = A_PICK + NumObjects : A_PLACE
    A_PLACE = A_IDENTIFY + NumObjects : A_PERCEIVE
    A_PERCEIVE = A_PLACE + 1 : A_PERCEIVE + NumTables
    */
    
//    cout << "Step -- ";
//    DisplayAction(action, cout);
    
    // NAVIGATION
    // TODO: restrict navigation from/to some poses?
    if (action < A_PICK){
        reward = reward_move;

        if(action < 0) return false;
        
        int currentPose = mobipickState.AgentPose;
        bool reach = false;
        
        //Evaluate the 3 basic poses
        switch(action){
            case P_TERMINAL:
                if(currentPose == P_OTHER) reach = true;
                break;
            case P_OTHER:
                reach = true;
                break;
            case P_BASKET:
                if(currentPose == P_OTHER) reach = true;
                break;
        }
        
        //P_TABLE poses are available only from P_OTHER and their corresponding P_NEAR
        if(action >= P_TABLE && action < P_TABLE + NumTables){
            int table = action - P_TABLE;
            if(currentPose == P_OTHER || currentPose == P_NEAR + table)
                reach = true;
        }
        
        //Near poses are available only from P_OTHER and their corresponding P_TABLE
        if(action >= P_NEAR && action < P_NEAR + NumTables){
            int table = action - P_NEAR;
            if(currentPose == P_OTHER || currentPose == P_TABLE + table)
                reach = true;
        }
        
        //Move agent only if position is reachable
        if(reach){
            mobipickState.AgentPose = action;
        }
        
        //Verify terminal condition at correct pose
        if(mobipickState.AgentPose == P_TERMINAL){
            /*int countCyls = 0;
            for(auto o : mobipickState.Basket.Objects){
                if(o.type == F_CYL) countCyls++;
            }
            
            if(countCyls >= ReqCyls){
                reward = terminal_success;
                return true;
            }*/                    
            if(mobipickState.Basket.Objects.size() > 0)
                reward = terminal_success;
            else
                reward =  terminal_fail;
            return true;
        }
        
        return false; //End here
    }

    // GRASP OBJECT 0 - NumObjects.  Can only grasp at tables, not at basket
    if(action < A_IDENTIFY){
        reward = reward_grasp;
        observation = O_FAIL; //Assume fail, e.g. wrong pose, grasp failure, etc.
        
        //If holding something fail immediately
        if(mobipickState.grasping){
            reward = reward_bad_grasp;
            return false;            
        }
        
        //If not holding, locate object id
        int obj = action - A_PICK; //Get object id
        int obj_pos = 0;
        bool found = false;
        
        //Find table with object
        int table_id = 0;
        for(auto& t : mobipickState.Tables){
            obj_pos = 0;
            for(auto& o : t.Objects){
                if(o.id == obj){
                    table_id = t.id;
                    found = true;                 
                }
                if(found) break;
                obj_pos++;
            }
            if(found) break;
        }
        
        //cout << "Object " << obj << " found on table " << table_id << " at pos = " << obj_pos << endl;
        
        //Attempt if pose == at table              
        assert(obj < NumObjects);
        
        //Verify pose allows grasping object id
        if(mobipickState.AgentPose == P_TABLE + table_id && found){                        
            //cout << "Attempting to grab obj " << obj << endl;
                        
            //If Position is not known, fail grasp
            if(!mobipickState.Tables[table_id].Objects[obj_pos].PosKnown){
                return false;
            }
            
            double p_grasp = MOBIPICK::PROB_GRASP_OTHER; //Assume not a cyl
            //If object IS a cylinder, adjust simulated grasping probability
            if(mobipickState.Tables[table_id].Objects[obj_pos].type == F_CYL) p_grasp = MOBIPICK::PROB_GRASP;
            
            if(Bernoulli(p_grasp)){
                observation = O_SUCCESS;
                
                //TODO: create POP function that returns a pointer or Object
                mobipickState.inGrasp.copy(mobipickState.Tables[table_id].Objects[obj_pos]); //Create copy of obj in grasp
                mobipickState.grasping = true;
                
                mobipickState.Tables[table_id].Objects.erase(mobipickState.Tables[table_id].Objects.begin() + obj_pos); //Remove from table
                
                //cout << "Now grasping obj " << mobipickState.inGrasp->id << endl;
            }
            else{
                //If grasping fails, receive small punishment (i.e. more likely for non-cyls)
                reward = reward_grasp_fail;
            }
        }
        
        return false;
    }
    
    //IDENTIFY: (Attempt to) Scan a distinct object and approximate its type: F_CYL, F_NOCYL
    // Identify works for an object when a) pose = at/near table, and b) obj_pose is 'known'
    if(action < A_PLACE){
        //Get obj id
        int obj  = action - A_IDENTIFY;        
        reward = reward_identify;
        
        //Find table with object
        int table_id = 0;
        bool found = false;
        int obj_pos = 0;
        
        for(auto t : mobipickState.Tables){
            table_id = t.id;
            obj_pos = 0;
            for(auto o : t.Objects){
                if(o.id == obj){                    
                    found = true;
                }
                if(found) break;
                obj_pos++;
            }
            if(found) break;
        }
        
        //Assume failure
        observation = O_FAIL;
        
        if(!found) return false; //e.g. object is in grasp, or in basket
        
        //Object is on table_id, verify correct pose        
        if(mobipickState.AgentPose != P_TABLE + table_id && mobipickState.AgentPose != P_NEAR + table_id){
            return false;
        }
                
        //Receive observation from sensor
        //cout << "Identifying pos = " << obj_pos << ", obj " << mobipickState.Tables[table_id].Objects[obj_pos].id << endl;
        observation = Identify(mobipickState.Tables[table_id].Objects[obj_pos]);
        
        if(observation == O_FAIL) return false;
        
        ///If we made it this far, everything is in order
        mobipickState.Tables[table_id].Objects[obj_pos].measured++;
        //DisplayObservation(mobipickState, observation, cout);

        //Compute Likelihoods from observation
        double efficiency = MOBIPICK::IDENTIFY_ACC;
        if (observation == O_CYL) {
            mobipickState.Tables[table_id].Objects[obj_pos].count++;
            mobipickState.Tables[table_id].Objects[obj_pos].LikelihoodCyl *= efficiency;
            mobipickState.Tables[table_id].Objects[obj_pos].LikelihoodNotCyl *= 1.0 - efficiency;
        } else {
            mobipickState.Tables[table_id].Objects[obj_pos].count--;
            mobipickState.Tables[table_id].Objects[obj_pos].LikelihoodNotCyl *= efficiency;
            mobipickState.Tables[table_id].Objects[obj_pos].LikelihoodCyl *= 1.0 - efficiency;
        }

        //Update target probability
        double denom = (0.5 * mobipickState.Tables[table_id].Objects[obj_pos].LikelihoodCyl) + (0.5 * mobipickState.Tables[table_id].Objects[obj_pos].LikelihoodNotCyl);
        mobipickState.Tables[table_id].Objects[obj_pos].ProbCyl = (0.5 * mobipickState.Tables[table_id].Objects[obj_pos].LikelihoodCyl) / denom;
        
        return false;
    }

    //PLACE: Attempt to place grasped object on an empty space on a table or basket
    //TODO: add ability to sense empty space and therefore chance to fail by placing on occupied space
    if(action < A_PERCEIVE){
        observation = O_FAIL;
        reward = reward_place;
        
        int table_id;
        
        //If no object, place fails
        if(!mobipickState.grasping){
            reward = reward_bad_place;
            observation = O_FAIL;
            
            return false;
        }
            
        //If at basket, verify terminal state
        if(mobipickState.AgentPose == P_BASKET){
            //Reward accordingly
            if(mobipickState.inGrasp.type == F_CYL) reward = reward_good; //TODO: switch to assumed type
            else reward = reward_bad;
            
            //Transfer object to basket
            MOBIPICK_STATE::OBJECT o(mobipickState.inGrasp);
            mobipickState.Basket.Objects.push_back(o);            
            mobipickState.grasping = false;
            
            observation = O_SUCCESS;
            
            return false;
        }
            
        //If AT a table, placing always succeeds
        if(mobipickState.AgentPose >= P_TABLE && mobipickState.AgentPose < P_NEAR){
            table_id = mobipickState.AgentPose - P_TABLE;
            
            //Transfer object to table
            MOBIPICK_STATE::OBJECT o(mobipickState.inGrasp);
            mobipickState.Tables[table_id].Objects.push_back(o);            
            mobipickState.grasping = false;
            
            observation = O_SUCCESS;
        }
        
        //Every other pose just fails
        
        return false;
    }
    
    //PERCEIVE: 'sense' a nearby table and update the poses of all objects on it
    //It's always available, but only returns correct information near/at a table
    if(action >= A_PERCEIVE){
        reward = reward_perceive;
                
        observation = Perceive(mobipickState.AgentPose);
        
        //Wrong pose or accuracy fail
        if(observation == O_NONE) return false;
        
        //Otherwise, observation is a table
        int table_id = observation - O_TABLE;        
        double efficiency = MOBIPICK::PERCEIVE_ACC;        
        
        //Update knowledge about the position of each object on table        
        
        for(auto& o : mobipickState.Tables[table_id].Objects){
            o.LikelihoodPos *= efficiency;
            o.LikelihoodNotPos *= 1.0 - efficiency;

            //Update probability
            double denom = (0.5 * o.LikelihoodPos) + (0.5 * o.LikelihoodNotPos);
            o.ProbPos = (0.5 * o.LikelihoodPos) / denom;
            
            //Update known position
            double binEntropy = -1*o.ProbPos*log2(o.ProbPos) - (1-o.ProbPos)*log2(1-o.ProbPos);
            if(binEntropy <= MOBIPICK::IDENTIFY_THRESHOLD){
                o.PosKnown = true;
            }
        }
        
        return false;
    }

    return false;
}


// Mobipick local transformations: change one object from cyl to no_cyl
bool MOBIPICK::LocalMove(STATE& state, const HISTORY& history, int stepObs, const STATUS& status) const{
    
    MOBIPICK_STATE& mobipickState = safe_cast<MOBIPICK_STATE&>(state);
        
    int table = Random(NumTables);
    
    //If this table is now empty, local move is not valid
    if(mobipickState.Tables[table].Objects.size() == 0){
        return false;        
    }
    
    int obj = Random(mobipickState.Tables[table].Objects.size());
    
    mobipickState.Tables[table].Objects[obj].type = !mobipickState.Tables[table].Objects[obj].type;
    
    int action = history.Back().Action;

    //IDENTIFY: validate id observations
    if (action >= A_IDENTIFY && action < A_PLACE) {
        
        //Object checked, and observation
        obj = action - A_IDENTIFY;
        int realObs = history.Back().Observation;
        
        bool found = false;
        int obj_pos = 0;
        for(auto t : mobipickState.Tables){
            table = t.id;
            obj_pos = 0;
            for(auto o : t.Objects){
                if(o.id == obj) found = true;
                if(found) break;
                obj_pos++;
            }
            if(found) break;
        }
        
        //Get new observation
        int newObs = Identify(mobipickState.Tables[table].Objects[obj_pos]);

        //If observations do not match, reject
        if (newObs != realObs)
            return false;
    }

    return true;
}

/* Fast PGS for Rollout policy
 * Simplified PGS point count by using only the specific action changes
 */
double MOBIPICK::PGS_RO(STATE& oldstate, STATE& state, int action, double oldpgs) const
{
    double points = 0.0;
    double oldpoints = 0.0;

    //1. Cast to cellarstate
    MOBIPICK_STATE& mobipickState = safe_cast<MOBIPICK_STATE&>(state);
    MOBIPICK_STATE& oldmobipickState = safe_cast<MOBIPICK_STATE&>(oldstate);
        
    //1. Grasp: + if obj was in fact picked, and is cyl and has known pos
    if(action >= A_PICK && action < A_IDENTIFY && mobipickState.grasping){
        //If object grasped is likely a cyl AND has known position, give bonus
        if( BinEntropyCheck(mobipickState.inGrasp.ProbCyl) && mobipickState.inGrasp.PosKnown ) points += PGS_pick_pos;
    }
    
    //2. Place in Basket (if place and object held WAS good/bad...reward)
    else if(action >= A_PLACE && action < A_PERCEIVE){
        if(mobipickState.AgentPose == P_BASKET && oldmobipickState.grasping){
            if(oldmobipickState.inGrasp.type == F_CYL) points += PGS_good_obj;
            else points += PGS_bad_obj;
        }
    }
    
    //3. Identify: neg. points if object fails binEntropyCheck
    else if (action >= A_IDENTIFY && action < A_PLACE){
        int obj = action - A_IDENTIFY;
        
        //find object in table
        int table_id, o_pos;        
        bool found = false;
        
        for(auto t : mobipickState.Tables){
            table_id = t.id;
            o_pos = 0;            
            for(auto o : t.Objects){
                if(o.id == obj) found = true;
                if(found) break;
                o_pos++;
            }
            if(found) break;
        }
        
        if(found){        
            if(!BinEntropyCheck(mobipickState.Tables[table_id].Objects[o_pos].ProbCyl)) points += PGS_uncertain;                        
            if(!BinEntropyCheck(oldmobipickState.Tables[table_id].Objects[o_pos].ProbCyl)) oldpoints += PGS_uncertain;            
        }
    }
    
    //4. Perceive: neg points for objects with unknown pos
    else if(action >= A_PERCEIVE){
        int table_id = -1;
        if(mobipickState.AgentPose >= P_TABLE && mobipickState.AgentPose < P_NEAR) table_id = mobipickState.AgentPose - P_TABLE;
        if(mobipickState.AgentPose >= P_NEAR && mobipickState.AgentPose < P_NEAR + NumTables) table_id = mobipickState.AgentPose - P_NEAR;
        
        //Action has an effect only in the above poses
        if(table_id >= 0 && table_id < NumTables){
            for(int o_pos=0; o_pos < mobipickState.Tables[table_id].Objects.size(); o_pos++){
                if(!mobipickState.Tables[table_id].Objects[o_pos].PosKnown) points += PGS_uncertain;
                if(!oldmobipickState.Tables[table_id].Objects[o_pos].PosKnown) oldpoints += PGS_uncertain;
            }
        }
    }
    //Update difference for current action
    double result = oldpgs - oldpoints + points;

    return result;
}

/*
 * PGS point count
    +0.5 pick cyl with good pos
    +1 good cyls in basket
    -1 bad/non cyls in basket
    
    -1 unknown objs

 */
double MOBIPICK::PGS(STATE& state) const
{
    double points = 0.0;
    
    //1. Cast
    MOBIPICK_STATE& mobipickState = safe_cast<MOBIPICK_STATE&>(state);

    //2. If object in grasp
    if(mobipickState.grasping){
        //If object in grasp is likely a cyl, give bonus
        if( BinEntropyCheck(mobipickState.inGrasp.ProbCyl) ) points += PGS_pick_pos;
    }
    
    //3. Points for objects in basket
    double p_target, binaryEntropy;
    for(auto o : mobipickState.Basket.Objects){
        if( o.type == F_CYL )
            points += PGS_good_obj;
        else
            points += PGS_bad_obj;
    }

    //3. Negative points for unidentified features (type AND position)
    for(auto t : mobipickState.Tables){
        for(auto o : t.Objects){
            if(!BinEntropyCheck(o.ProbCyl)) points += PGS_uncertain;
            if(!o.PosKnown) points += PGS_uncertain;
        }
    }

    return points;
}

// PGS Rollout policy
void MOBIPICK::GeneratePGS(const STATE& state, const HISTORY& history,
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
/*
    cout << "found " << legal.size() << " rollout actions with PGS = " << max_v << endl;
    for(auto a : legal)
        DisplayAction(a,cout);
    cout << endl;*/
}

/*
 * Legal actions following PGS policy: avoid actions that do not reduce uncertainty
*/

void MOBIPICK::PGSLegal(const STATE& state, const HISTORY& history,
                      vector<int>& legal, const STATUS& status) const
{
    const MOBIPICK_STATE& mobipickState = safe_cast<const MOBIPICK_STATE&>(state);

    //Navigate to/from restricted poses
    int currentPose = mobipickState.AgentPose;
    bool identify = false;
    bool place = false;
    bool perceive = false;
    
    //This pose is always available
    legal.push_back(P_OTHER);
    
    //In P_OTHER ALL poses are available
    if(currentPose == P_OTHER) for(int a=0; a<NumPoses; a++) legal.push_back(a);
            
    //At P_TABLE:
    // Nav. only to P_NEAR and P_OTHER
    // Pick
    // Place
    // Identify
    // Perceive
    if(currentPose >= P_TABLE && currentPose < P_TABLE + NumTables){
        int table_id = currentPose - P_TABLE;
        
        legal.push_back(P_NEAR + table_id); //Navigate to NEAR this table
        
        //Add all pick actions at this table, for active and known objs
        for(auto o : mobipickState.Tables[table_id].Objects){
            if(o.active && o.PosKnown) legal.push_back(A_PICK + o.id);
        }
        
        place = true;
        perceive = true;
        identify = true;
    }
        
    //At P_NEAR, only P_TABLE and P_OTHER are available, + perceive and identify
    if(currentPose >= P_NEAR && currentPose < P_NEAR + NumTables){
        legal.push_back(P_TABLE + currentPose - P_NEAR);
        perceive = true;
        identify = true;
    }

    
    //Identify active and unidentified objects
    if(identify)
        for(auto t : mobipickState.Tables){
            for(auto o : t.Objects){
                if(o.active && o.PosKnown) legal.push_back(A_IDENTIFY + o.id);
            }
        }

    //Place
    if(place || currentPose == P_BASKET) legal.push_back(A_PLACE);
    
    //Perceive tables
    if(perceive) legal.push_back(A_PERCEIVE);

    /*
    //Navigation is always available
    for(int a=0; a < NumPoses; a++)
        legal.push_back(a);

    //If agent is at a table, these actions are allowed:
    if(mobipickState.AgentPose >= P_TABLE && mobipickState.AgentPose < P_NEAR){
    
        int table_id = mobipickState.AgentPose - P_TABLE;

        for(auto o : mobipickState.Tables[table_id].Objects){
            //Pick, for active objects with known pos
            if(o.active && o.PosKnown) legal.push_back(A_PICK + o.id);
            
            //Identify, for active and unidentified objects with known pos        
            if(o.active && !BinEntropyCheck(o.ProbCyl) && o.PosKnown)
                legal.push_back(A_IDENTIFY + o.id);
        }

    }
    
    //If at table or basket and holding, also allow place
    if(mobipickState.AgentPose >= P_TABLE && mobipickState.AgentPose < P_NEAR || mobipickState.AgentPose == P_BASKET ){
        if(mobipickState.inGrasp != NULL){
            legal.push_back(A_PLACE);
        }
    }
    
    //Perceive tables in valid positions
    if(mobipickState.AgentPose >= P_TABLE && mobipickState.AgentPose < P_NEAR + NumTables)
        legal.push_back(A_PERCEIVE);
    */
}


void MOBIPICK::GenerateLegal(const STATE& state, const HISTORY& history,
                           vector<int>& legal, const STATUS& status) const
{
    const MOBIPICK_STATE& mobipickState = safe_cast<const MOBIPICK_STATE&>(state);
  
    //Navigation is always available
    for(int a=0; a < NumPoses; a++)
        legal.push_back(a);

    //Pick is available at a table
    if(mobipickState.AgentPose >= P_TABLE && mobipickState.AgentPose < P_NEAR){
        int table_id = mobipickState.AgentPose - P_TABLE;
        for(auto o : mobipickState.Tables[table_id].Objects){
            if(o.active && o.PosKnown) legal.push_back(A_PICK + o.id);
        }
    }
    
    //Identify is always available, but use for active and unidentified objects
    for(auto t : mobipickState.Tables){
        for(auto o : t.Objects){
            if(o.active && o.PosKnown) legal.push_back(A_IDENTIFY + o.id);
        }
    }

    //Always allow place?
    legal.push_back(A_PLACE);
    
    //Perceive tables anytime
    legal.push_back(A_PERCEIVE);

}

/*
  Preferred actions RO policy
  ** NOT USED IN MOBIPICK **
*/
void MOBIPICK::GeneratePreferred(const STATE& state, const HISTORY& history,
                               vector<int>& actions, const STATUS& status) const
{
    GenerateLegal(state, history, actions, status);
}

///// Domain specific //////

/*
 * Identify object:
 *
 * With p = IDENTIFY_ACC, recognize object as cylinder correctly
 * With failure object is misclassified
 * */
int MOBIPICK::Identify(MOBIPICK_STATE::OBJECT& o) const {
    int obs;
        
    //Fail pose uncertainty is high
    if(!o.PosKnown){
        obs = O_FAIL;
    }
    else if (Bernoulli(MOBIPICK::IDENTIFY_ACC)){ //If OK, 'observe' true type
            obs = (o.type == F_CYL) ? O_CYL : O_NOCYL;
        }
        else{ //If fail, observe wrong type
            obs = (o.type == F_CYL) ? O_NOCYL : O_CYL;
        }

    return obs;
}


/*
 * Perceive (poses of objects on) table with p = PERCEIVE_ACC
 * 
 * Works only if pose = at table or near table
 * 
 * Returns id of table perceived, if fail returns NONE
 * 
 */
int MOBIPICK::Perceive(int pose) const{
    int obs = O_NONE;
    int table_id;
    int proximity = 0; //0 = far, 1 = near, 2 = at
    
    if(pose >= P_TABLE && pose < P_TABLE + NumTables){
        proximity = 2;
        table_id = pose - P_TABLE;
    }
    if(pose >= P_NEAR && pose < P_NEAR + NumTables){
        proximity = 1;
        table_id = pose - P_NEAR;
    }

    if(proximity > 0){ //TODO: maybe use different probs if at or near
        if (Bernoulli(MOBIPICK::PERCEIVE_ACC)){ //If OK, 'observe' table id
            obs = O_TABLE + table_id;
        }        
    }

    return obs;
}

/*
 * Return whether value p satisfies the entropy restriction < BIN_ENTROPY_LIMIT in a Bernoulli distribution
 * 
 * true if check is passed, meaning the value satisfies the uncertainty threshold.
 * 
 * */
bool MOBIPICK::BinEntropyCheck(double p) const {
    double binaryEntropy = -1*p*log2(p) - (1-p)*log2(1-p);
    return (binaryEntropy <= MOBIPICK::BIN_ENTROPY_LIMIT);
}

/*
 * OUTPUT FUNCTIONS
 */

void MOBIPICK::DisplayBeliefs(const BELIEF_STATE& beliefState,
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

//Return text representation of pose
std::string MOBIPICK::Pose2Str(int pose) const{
    std::ostringstream oss;
    
    if(pose == P_TERMINAL)
        oss << "Terminal";
    else if(pose == P_BASKET)
        oss << "Basket";
    else if (pose == P_OTHER)
        oss << "Other";
    else if(pose >= P_TABLE && pose < P_NEAR)
        oss << "Table " << pose - P_TABLE;
    else if(pose >= P_NEAR && pose < P_NEAR + NumTables)
        oss << "Near Table " << pose - P_NEAR;
    else
        oss << "error? (" << pose << ")";
    
    return oss.str();
}

void MOBIPICK::DisplayState(const STATE& state, std::ostream& ostr) const
{
    const MOBIPICK_STATE& mobipickState = safe_cast<const MOBIPICK_STATE&>(state);
    ostr << endl;
        
    std::string pose_str = Pose2Str(mobipickState.AgentPose);
    
    ostr << "Agent pose: " << pose_str << endl;
    
    ostr << "Grasping: ";
    
    if(mobipickState.grasping){
        ostr << " Obj. " << mobipickState.inGrasp.id << ", Type = ";
        if(mobipickState.inGrasp.type == F_CYL)
            ostr << " Cylinder";
        else
            ostr << " Not cylinder";
        
        cout << ", P(cyl) = " << mobipickState.inGrasp.ProbCyl;
    }
    else{
        ostr << " Nothing";
    }
    cout << endl;
    
    int cyl = 0;
    int noCyl = 0;
    for(auto o : mobipickState.Basket.Objects){
        if(o.type == F_CYL) cyl++;
        else noCyl++;
    }
    
    //ostr << "Basket contents: " << cyl << " / " << ReqCyls << " cylinders, " << noCyl << " other." << endl;
    //Now w/o req cyls
    ostr << "Basket contents: " << cyl << " cylinders, " << noCyl << " other." << endl;
    
    //ostr << "Goal: " << ReqCyls << " cylinders" << endl;
    
    //Display content of all tables
    ostr << "Table\tO. ID\tP(Pos)\tP.Known\tP(Cyl)\tType" << endl;
    for(auto t : mobipickState.Tables){
        for(auto o : t.Objects){
            ostr << t.id << "\t";
            ostr << o.id << "\t";
            ostr << std::setprecision(4) << o.ProbPos << "\t";
            if(o.PosKnown)
                ostr << "Y\t";
            else
                ostr << "N\t";
            ostr << std::setprecision(4) << o.ProbCyl << "\t";
            
            if(o.type == F_CYL) ostr << " C";
            else ostr << " N";
            
            ostr << endl;
        }
    }

    ostr << endl;
}

void MOBIPICK::DisplayObservation(const STATE& state, int observation, std::ostream& ostr) const
{
    ostr << "Observed ";
    switch (observation)
    {
        case O_EMPTY:
            ostr << "empty space" << endl;
            break;
        case O_NONE:
            ostr << "nothing." << endl;
            break;
        case O_CYL:
            ostr << "a cylinder!" << endl;
            break;
        case O_NOCYL:
            ostr << "something, but not a cylinder." << endl;
            break;
        case O_SUCCESS:
            ostr << "success!" << endl;
            break;
        case O_FAIL:
            ostr << "failure" << endl;
            break;
    }

    if(observation >= O_TABLE) {
        int n = observation - O_TABLE;
        ostr << "Table " << n << endl;
    }
}

void MOBIPICK::DisplayAction(int action, std::ostream& ostr) const
{
    /*
        A_NAVIGATION = 0 : NumPoses
        A_PICK = A_NAVIGATION + NumPoses : A_IDENTIFY
        A_IDENTIFY = A_PICK + NumObjects : A_PLACE
        A_PLACE = A_IDENTIFY + NumObjects : A_PERCEIVE
        A_PERCEIVE = A_PLACE + 1 : A_PERCEIVE + NumTables
     */
    
    ostr << "A: ";
    if (action < A_PICK){
        std::string pose_str = Pose2Str(action);            
        ostr << "Move to pose [" << pose_str << "]" << endl;
    }
    else if(action >= A_PICK && action < A_IDENTIFY)
        ostr << "Pick object " << action - A_PICK << endl;
    else if(action >= A_IDENTIFY && action < A_PLACE)
        ostr << "Identify object " << action - A_IDENTIFY << endl;
    else if(action >= A_PLACE && action < A_PERCEIVE)
        ostr << "Place" << endl;
    else if(action >= A_PERCEIVE)
        ostr << "Perceive" << endl;

}

