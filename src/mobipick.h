/*
 * Mobipick Domain v1.0
 *
 * A simplified model of the Mobipick robot in an object grasping domain.
 * 
 * Object locations/IDs are known but their types and exact positions are initially unknown.  Positions are needed to grasp and identify objects.
 *
 * The goal is to move a number of cylinders/good objects to a basket, and transitioning to the TERMINAL pose.
 * 
 * By Juan Carlos Saborio
 * DFKI Lab Niedersachsen, 2021
 * 
 * */

#ifndef MOBIPICK_H
#define MOBIPICK_H

#include "simulator.h"
#include <cstring>
#include <sstream>

struct MOBIPICK_PARAMS : PROBLEM_PARAMS{
    //Default values are Demo 1
    int cylinders = 3;
    int objects = 6;
    int tables = 2;
    int reqCyls = 3;
    
    double activation = -6.0; //IRE activation threshold
    double discount = 0.95; //POMDP discount
    double fDiscount = 0.5; //IRE feature discount
    double identify = 0.85; //Object identification accuracy
    double perceive = 0.75; //Table perceive accuracy
    double grasping = 0.85; //Grasping success probability (cylinders only)
    double grasping_other = 0.45; //Grasping success prob. for non cylinders
    double entropy = 0.5; //PGS entropy
    double PGSAlpha = 10; //PGS scaling factor
    double transitionRate = 1.0; //"Learning rate" for values in f-table upon transitions
    
    double moving = 0.25; //Prob of obstacles/people/etc moving.  Not yet implemented

    MOBIPICK_PARAMS() : cylinders(3), objects(6), tables(2), reqCyls(3),
                    activation(-6.0), discount(0.95), fDiscount(0.5), 
                    identify(0.85), perceive(0.75), grasping(0.85), moving(0.25),
                    entropy(0.5), PGSAlpha(10){}
};

//TODO: Add array destructor
class MOBIPICK_STATE : public STATE
{
public:
    int AgentPose;    
    
    //Currently not in use
    struct LOCATION{
        int x;
        int y;
    };

    struct OBJECT{
        //Ground truth:
        int id;
        LOCATION loc; //Location within table
        int type; //{cyl, no cyl}
        
        //POMDP:
        int measured;
        int count;
        
        //Type uncertainty
        double ProbCyl;        
        double LikelihoodCyl;
        double LikelihoodNotCyl;
        
        //Position uncertainty
        bool PosKnown;
        double ProbPos; //p(known)
        double LikelihoodPos;
        double LikelihoodNotPos;
        
        bool active;
        
        OBJECT(){}
        
        OBJECT(const MOBIPICK_STATE::OBJECT& o){
            id = o.id;
            loc.x = o.loc.y;
            loc.y = o.loc.y;
            type = o.type;
            measured = o.measured;
            count = o.count;
            
            ProbCyl = o.ProbCyl;            
            LikelihoodCyl = o.LikelihoodCyl;
            LikelihoodNotCyl = o.LikelihoodNotCyl;
            
            PosKnown = o.PosKnown;
            ProbPos = o.ProbPos;
            LikelihoodPos = o.LikelihoodPos;
            LikelihoodNotPos = o.LikelihoodNotPos;
            
            active = o.active;
        }
        
        void copy(const MOBIPICK_STATE::OBJECT& o){
            id = o.id;
            loc.x = o.loc.y;
            loc.y = o.loc.y;
            type = o.type;
            measured = o.measured;
            count = o.count;
            
            ProbCyl = o.ProbCyl;            
            LikelihoodCyl = o.LikelihoodCyl;
            LikelihoodNotCyl = o.LikelihoodNotCyl;
            
            PosKnown = o.PosKnown;
            ProbPos = o.ProbPos;
            LikelihoodPos = o.LikelihoodPos;
            LikelihoodNotPos = o.LikelihoodNotPos;
            
            active = o.active;
        }
    };
    
    OBJECT inGrasp;
    bool grasping = false;
    
    struct TABLE
    {
        int id;
        LOCATION loc; //will not change
        int sizeX, sizeY;
        std::vector<OBJECT> Objects;
    };
    std::vector<TABLE> Tables;
    
    //Obstacles not yet implemented
    struct OBSTACLE
    {
        int id;
        int pose;
    };
    std::vector<OBSTACLE> Obstacles;
    
    struct BASKET_S
    {
        int pose;
        int capacity; //Make goal more challenging by limiting size
        LOCATION loc;
        std::vector<OBJECT> Objects; //Maybe just keep count and remove object structs
    };
    BASKET_S Basket;

    void activateFeature(int feature, bool status);
    
    ~MOBIPICK_STATE();
};

class MOBIPICK : public SIMULATOR{
public:

    MOBIPICK(PROBLEM_PARAMS& problem_params);

    //Core Simulator functions
    virtual STATE* Copy(const STATE& state) const;
    virtual void Validate(const STATE& state) const;
    virtual STATE* CreateStartState() const;
    virtual void FreeState(STATE* state) const;
    virtual bool Step(STATE& state, int action,
                      int& observation, double& reward) const;

    /*** PGS functions ***/
    //Uses regular POMCP Step
    bool StepNormal(STATE& state, int action,
                    int& observation, double& reward) const;
    //Step with PGS rewards
    bool StepPGS(STATE& state, int action,
                 int& observation, double& reward) const;
    // Simple Step (transition only)
    bool SimpleStep(STATE& state, int action) const;
    //PGS Rollout policy
    void GeneratePGS(const STATE& state, const HISTORY& history,
                     std::vector<int>& legal, const STATUS& status) const;

    //Compute PGS value
    double PGS(STATE& state) const;
    double PGS_RO(STATE& oldstate, STATE& state, int action, double oldpgs) const;  //PGS for rollouts

    ///// Incremental refinement /////
    std::vector<FTABLE::F_ENTRY>& getInitialFTable() const {}
    void initializeFTable(FTABLE& ftable) const;

    /********************************/

    /* Virtual functions from Simulator class */
    void GenerateLegal(const STATE& state, const HISTORY& history,
                       std::vector<int>& legal, const STATUS& status) const;
    void PGSLegal(const STATE& state, const HISTORY& history,
                  std::vector<int>& legal, const STATUS& status) const;
    void GeneratePreferred(const STATE& state, const HISTORY& history,
                           std::vector<int>& legal, const STATUS& status) const;
    virtual bool LocalMove(STATE& state, const HISTORY& history,
                           int stepObservation, const STATUS& status) const;

    virtual void DisplayBeliefs(const BELIEF_STATE& beliefState,
                                std::ostream& ostr) const;
    virtual void DisplayState(const STATE& state, std::ostream& ostr) const;
    virtual void DisplayObservation(const STATE& state, int observation, std::ostream& ostr) const;
    virtual void DisplayAction(int action, std::ostream& ostr) const;

protected:

    //Observations
    enum
    {
        O_NONE,
        O_EMPTY,
        O_CYL,
        O_NOCYL,
        O_SUCCESS,
        O_FAIL,
        O_TABLE
    };

    //Poses
    enum{
        P_TERMINAL,
        P_BASKET,
        P_OTHER
    };
    
    //Feature categories
    enum{
        F_CYL,
        F_NOCYL,
        F_OBSTACLE
    };

    void InitGeneral();
    void Init_Demo1();
    
    bool BinEntropyCheck(double p) const; //Verify that p satisfies threshold
    
    /* Mobipick domain functions */
    int Perceive(int pose) const; //Get poses of all objects in nearby table
    int Identify(MOBIPICK_STATE::OBJECT& o_ptr) const; //Get type of object scanned
    std::string Pose2Str(int pose) const; //Return text representation of pose
    
    /*
     * Domain information/representation
     * 
     */    
    double IDENTIFY_ACC; //0 - 1 accuracy for obj identify
    double IDENTIFY_THRESHOLD = 0.5;
    
    double PERCEIVE_ACC; //0 - 1 accuracy for perceive table
    double PROB_GRASP; //0 - 1 grasping success probability
    double PROB_GRASP_OTHER; //Grasp prob. for non cylinders
    double PROB_MOVING;
    double BIN_ENTROPY_LIMIT; //0.5 or set to preference
    double ACTIVATION_THRESHOLD;
    double PGSAlpha; //PGS scaling factor
    
    std::vector< std::vector<int> > initTables; //Type array to setup distinct problem layouts
    
    //NumActions and NumObservations inherited from Simulator
    int ReqCyls; //Goal criteria
    int Size, NumPoses, NumCylinders, NumObjects, NumTables, NumObstacles, NumFeatures;
    int A_NAVIGATION, A_PICK, A_PLACE, A_IDENTIFY, A_PERCEIVE; //Markers for the beginning of action group
    int P_TABLE, P_NEAR, P_OBJ; //Markers for poses: At table, Near table, Grasp object
    
    double HalfEfficiencyDistance; //Are we using a rocksample-style sensor?
    
    /*
     * PGS point distribution
     * 
     */
    double PGS_pick_pos = 0.5;
    double PGS_good_obj = 1;
    double PGS_bad_obj = -1;
    double PGS_uncertain = -0.5;

private:
    mutable MEMORY_POOL<MOBIPICK_STATE> MemoryPool;
};

#endif
