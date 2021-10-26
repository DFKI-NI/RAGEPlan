#ifndef SIMULATOR_H
#define SIMULATOR_H

#include "history.h"
#include "node.h"
#include "utils.h"
#include <iostream>
#include <math.h>

#include "grid.h"

#include "ftable.h"

class BELIEF_STATE;

class STATE : public MEMORY_OBJECT
{
public:
	virtual void activateFeature(int feature, bool status){}
};

struct PROBLEM_PARAMS : MEMORY_OBJECT{
    std::string problem;
    std::string description;
};

class SIMULATOR
{
public:

    struct KNOWLEDGE
    {
        enum
        {
            PURE,
            LEGAL,
            SMART,
			PGS,
            NUM_LEVELS
        };

        KNOWLEDGE();
        
        int RolloutLevel; //Rollout Policy
        int TreeLevel; //NODE Initialization policy -- e.g. for preferred actions
        int SmartTreeCount; // If TreeLevel > SMART, use SmartCount
        double SmartTreeValue; // If TreeLevel > SMART, use SmartValue
        
        int Level(int phase) const
        {
            assert(phase < STATUS::NUM_PHASES);
            if (phase == STATUS::TREE)
                return TreeLevel;
            else
                return RolloutLevel;
        }
    };

    struct STATUS
    {
        STATUS();
        
        enum
        {
            TREE,
            ROLLOUT,
            NUM_PHASES
        };
        
        enum
        {
            CONSISTENT,
            INCONSISTENT,
            RESAMPLED,
            OUT_OF_PARTICLES
        };
        
        int Phase;
        int Particles;
    };
	 
	 /////////////////////////
	 //Incremental refinement
	 bool useFtable = false;
	 virtual std::vector<FTABLE::F_ENTRY>& getInitialFTable() const {}
	 virtual void initializeFTable(FTABLE& ftable) const {}
	 ////////////////////////
	 
    SIMULATOR();
    SIMULATOR(int numActions, int numObservations, double discount = 1.0);    
    virtual ~SIMULATOR();

    // Create start start state (can be stochastic)
    virtual STATE* CreateStartState() const = 0;

    // Free memory for state
    virtual void FreeState(STATE* state) const = 0;

    // Update state according to action, and get observation and reward. 
    // Return value of true indicates termination of episode (if episodic)
    virtual bool Step(STATE& state, int action, 
        int& observation, double& reward) const = 0;
        
    // Create new state and copy argument (must be same type)
    virtual STATE* Copy(const STATE& state) const = 0;
    
    // Sanity check
    virtual void Validate(const STATE& state) const;

    // Modify state stochastically to some related state
    virtual bool LocalMove(STATE& state, const HISTORY& history,
        int stepObs, const STATUS& status) const;

    // Use domain knowledge to assign prior value and confidence to actions
    // Should only use fully observable state variables
    void Prior(const STATE* state, const HISTORY& history, VNODE* vnode,
        const STATUS& status) const;

    // Use domain knowledge to select actions stochastically during rollouts
    // Should only use fully observable state variables
    int SelectRandom(const STATE& state, const HISTORY& history,
        const STATUS& status) const;

    // Generate set of legal actions
    virtual void GenerateLegal(const STATE& state, const HISTORY& history, 
        std::vector<int>& actions, const STATUS& status) const;

    // Generate set of preferred actions
    virtual void GeneratePreferred(const STATE& state, const HISTORY& history, 
        std::vector<int>& actions, const STATUS& status) const;

    // Relevance-planning functions
    // Generate set of relevant actions
    void GenerateRelevant(const STATE& state, const HISTORY& history,
        std::vector<int>& actions, const STATUS& status) const;
    virtual void PGSLegal(const STATE& state, const HISTORY& history,
        std::vector<int>& actions, const STATUS& status) const;
    // Generate set of PGS actions
    virtual void GeneratePGS(const STATE& state, const HISTORY& history,
                                   std::vector<int>& actions, const STATUS& status) const;

    // Textual display
    virtual void DisplayBeliefs(const BELIEF_STATE& beliefState, 
        std::ostream& ostr) const;
    virtual void DisplayState(const STATE& state, std::ostream& ostr) const;
    virtual void DisplayAction(int action, std::ostream& ostr) const;
    virtual void DisplayObservation(const STATE& state, int observation, std::ostream& ostr) const;
    virtual void DisplayReward(double reward, std::ostream& ostr) const;

    // Accessors
    void SetKnowledge(const KNOWLEDGE& knowledge) { Knowledge = knowledge; }
    int GetNumActions() const { return NumActions; }
    int GetNumObservations() const { return NumObservations; }
    bool IsEpisodic() const { return false; }
    double GetDiscount() const { return Discount; }
    double GetFDiscount() const { return fDiscount; }
    double GetRewardRange() const { return RewardRange; }
    double GetHorizon(double accuracy, int undiscountedHorizon = 100) const;
	 
	 //For initial state consistency
	 //virtual const GRID* GetGrid() const;
	 //virtual void SetGrid(const GRID* grid) const;
    
protected:

    int NumActions, NumObservations;
    double Discount, RewardRange;
    double fDiscount;
    KNOWLEDGE Knowledge;
};

#endif // SIMULATOR_H
