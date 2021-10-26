/*
 * Drone Domain v2.0
 *
 * An exploration drone moves around the woods and attempts to photograph Big Foot,
 * while avoiding trees and other animals
 *
 * The world is mapped as a grid, and creatures move around stochastically.  Plants don't move.
 * */

#ifndef DRONE_H
#define DRONE_H

#include "simulator.h"
#include "coord.h"
#include "grid.h"

struct DRONE_PARAMS : PROBLEM_PARAMS{
    int size = 3;
    int creatures = 3;
    int trees = 2;
    int targets = 1;
    int maxPhotos = 1;
    int photos = 1;
    double activation = -16.0;
    double discount = 0.95;
    double fDiscount = 0.3;
    double recognition = 0.9;
    double moving = 0.25;
    double entropy = 0.4;

    DRONE_PARAMS() : size(3), creatures(3), trees(3), targets(3),
                    maxPhotos(1), photos(1), activation(-16), discount(0.95),
                    fDiscount(0.3), recognition(0.9), moving(0.25), entropy(0.4){}
};

class DRONE_STATE : public STATE
{
public:
    COORD AgentPos;
    int TargetPhotosTaken;
    int NoTargetPhotosTaken;
    int PhotosTaken;
    int Battery;

    //Each entry has a prob. of being Big Foot
    struct P_ENTRY
    {
        //Ground truth & simulation
        bool Target; //Big Foot
        COORD Position;
        int type; //Tree, Bear, Big Foot

        //Proper state descriptors
        COORD ObservedPosition;
        double ProbPosition; //Probability that ObservedPosition is correct
        double LikelihoodTarget;
        double LikelihoodNotTarget;
        double ProbTarget;
        bool AssumedTarget;
        //Counts
        int numPhotos;
        int measured;
        int count;
        //Relevance
        bool active;
    };
    std::vector<P_ENTRY> Features;

    void activateFeature(int feature, bool status);
};

class DRONE : public SIMULATOR
{
public:

    DRONE(int size, int creatures, int trees, int targets, bool useTable = false);
    DRONE(PROBLEM_PARAMS& problem_params);

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
        O_EMPTY,
        O_NONE,
        O_TARGET,
        O_NOTARGET,
        O_FEATURE
    };

//    const int O_PEOPLE = 100;
//    int E_PEOPLE = 100;

    //Action categories
    enum
    {
        A_WAIT = 4,
        A_LEAVE,
        A_CHECK,
        A_IDENTIFY,
        A_PHOTO
    };

    //Feature categories
    enum{
        F_TREE,
        F_BEAR,
        F_BIGFOOT
    };

    void InitGeneral();
    void Init_3_3();  //Initial test setup
    void Init_3_3_3();
    void Init_5_8_8();

    int Observe(const DRONE_STATE &droneState, int cell) const;
    int Identify(const DRONE_STATE &droneState, int feature) const;
    void MoveFeature(DRONE_STATE &droneState, int feature) const;
    int IdentifyRoom(const DRONE_STATE &droneState, int room) const;
    int SelectTarget(const DRONE_STATE& droneState) const;

    /* Domain specific functions */
    bool BinEntropyCheck(double p) const;
    bool FeatureAt(const DRONE_STATE &droneState, int feature, const COORD &cell) const;
    bool EmptyCell(const DRONE_STATE &droneState, const COORD& coord) const;
    int PersonNumber(const DRONE_STATE& droneState, const COORD& coord) const;

    bool TargetIn(const DRONE_STATE &droneState, int cell) const;
    int NumPeopleInRoom(const DRONE_STATE& droneState, const COORD& coord) const;
    int FeatureInCell(const DRONE_STATE &droneState, int cell) const;
    int FeatureInCoord(const DRONE_STATE &droneState, const COORD& coord) const;
    void PeopleInCurrentRoom(const DRONE_STATE &droneState, std::vector<int>& people) const;

    GRID<int> Grid;


    /*
     * Domain (non state-dependent) information
     * */
    int PhotosNeeded; //Goal criteria
    int MaxPhotos; //Run out of "film", don't waste it
    double RECOGNITION_RATE; //0.0 - 1.0 recognition rate for identifier
    double PROB_MOVING;
    double BIN_ENTROPY_LIMIT; //0.5 or set to preference
    double ACTIVATION_THRESHOLD;

    struct P_INFO{
        char name[128];
    };
    std::vector<P_INFO> PeopleInfo;
    std::vector<int> Targets;
    int Target;
    int Size, NumCreatures, NumCells, NumTrees, NumTargets, NumFeatures;
    int E_CHECK, E_IDENTIFY, E_PHOTO; //Markers for the beginning of action group
    COORD StartPos;
    std::vector<COORD> FeaturePos; //Keep track of people
    double HalfEfficiencyDistance;

private:

    mutable MEMORY_POOL<DRONE_STATE> MemoryPool;
};

#endif