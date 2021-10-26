/* 
	Cellar Domain v1.0
	2018 by JCS, Uni-Osnabrueck.

	The "cellar" domain was designed to test the convergence of POMDP and POMDP-derived algorithms when
multiple "obstacles" are present, i.e. objects with interactions that do not directly contribute to solving the problem.

   This domain is an adaptation of rocksample, which correctly represents many robotic POMDP-planning tasks but incorrectly
assumes a minimalist model of the world (i.e. only rocks exist).  Cellar requires an agent to navigate and collect valuable bottles,
avoid non valuable bottles and leave (EAST) with *at least* one bottle.  Some bottles may be behind crates, which the agent may push in
different directions.  Other obstacles include shelves which cannot be moved.  Cellar differs from Rocksample in the following aspects:

	- Four parameters are needed: cellar[n,m,x,y] = nxn grid, m bottles, x shelves, y crates.
	- Four new actions are always available: PUSH + N, E, S, W.
	- Attempting to push non-crate objects yields a punishment of -10.
	- Each extra object also adds a CHECK action (in addition to each bottle).
	- Each step yields a small punishment of -1.
	
	It is worth noting the added actions, objects and observations create a much more complex POMDP than rocksample, and policies 
can become excessively long without the movement punishment.

	In this version, the initial location of all bottles and objects is known. Like rocksample, the agent determines the value of bottles and,
in addition, determines whether an object is a crate or a shelf.  The location of crates changes when pushed.

	Future versions may assume no initial knowledge whatsoever, so the agent must identify nearby objects/bottles first.
	
	We contend that keeping track of certain "irrelevant" objects creates an unnecessarily large search space, and that they can be safely ignored
without (significantly) affecting the convergence properties of the UCT algorithm.  This POMDP will allow us to verify this hypothesis and design
methods that can transfer to real life robots more easily.
*/

#ifndef CELLAR_H
#define CELLAR_H

#include "simulator.h"
#include "coord.h"
#include "grid.h"

struct CELLAR_PARAMS : PROBLEM_PARAMS{
	int size;
	int bottles;
	int crates;
	int shelves;
	double activation;
	double discount;
	double fDiscount;
	double entropy;
    double PGSAlpha = 10.0; //PGS Scaling factor
    double transitionRate = 1.0; //Ftable transition rate

    CELLAR_PARAMS() : size(5), bottles(2), crates(6), shelves(4),
        activation(-6), discount(0.95), fDiscount(0.3), entropy(0.5), PGSAlpha(10), transitionRate(1.0){}
};

class CELLAR_STATE : public STATE
{
public:

    COORD AgentPos;
    struct ENTRY
    {
        bool Valuable;
        bool Collected;
        int Count;    				// Smart knowledge
        int Measured; 				// Smart knowledge
        double LikelihoodValuable;	// Smart knowledge
        double LikelihoodWorthless;	// Smart knowledge
        double ProbValuable;		// Smart knowledge
    };
    std::vector<ENTRY> Bottles;	 
	 
	 struct OBJ_ENTRY
    {
        //bool Valuable;
		  COORD ObjPos;
        int Type;
        int Count;    				// Smart knowledge
        int Measured; 				// Smart knowledge
        double LikelihoodCrate;	// Smart knowledge
        double LikelihoodShelf;	// Smart knowledge
        double ProbCrate;		// Smart knowledge
		  int AssumedType;		// Assumptions
		  bool active;				//activate/deactivate based on value
    };
	 std::vector<OBJ_ENTRY> Objects;
    int Target; // Smart knowledge
	 int CollectedBottles;
	 
	 void activateFeature(int feature, bool status);
};

class CELLAR : public SIMULATOR
{
public:

    CELLAR(int size, int bottles, int shelves, int crates, bool useTable = false);
    CELLAR(PROBLEM_PARAMS& problem_params);

    virtual STATE* Copy(const STATE& state) const;
    virtual void Validate(const STATE& state) const;
    virtual STATE* CreateStartState() const;
    virtual void FreeState(STATE* state) const;
    virtual bool Step(STATE& state, int action,
        int& observation, double& reward) const;

	/*** Added by JCS to test PGS ***/
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
        E_NONE,
        E_GOOD,
        E_BAD,
		  E_SHELF,
		  E_CRATE
    };

	 //Objects
/*    enum
    {		  
        E_SHELF,
		  E_CRATE
    };
*/	 
	 //Action categories
    enum
    {        
		  A_SAMPLE = 4,
		  A_BOTTLECHECK,
		  A_OBJCHECK,
		  A_PUSHNORTH,
		  A_PUSHSOUTH,
		  A_PUSHEAST,
		  A_PUSHWEST		  
    };

    void InitGeneral();
	 void Init_5_1();
	 void Init_5_2();
	 void Init_Ftest();
	 void Init_Ftest2();
    void Init_7_8();
    void Init_11_11();
    int GetObservation(const CELLAR_STATE& cellarstate, int pos, int type) const;
    int SelectTarget(const CELLAR_STATE& cellarstate) const;
	 
	 bool CrateAt(const CELLAR_STATE& cellarstate, const COORD& coord) const;
	 bool ShelfAt(const CELLAR_STATE& cellarstate, const COORD& coord) const;
	 bool EmptyTile(const CELLAR_STATE& cellarstate, const COORD& coord) const;
	 bool FreeTile(const CELLAR_STATE& cellarstate, const COORD& coord) const;
	 int ObjectNumber(const CELLAR_STATE& cellarstate, const COORD& coord) const;

    GRID<int> Grid;
    std::vector<COORD> BottlePos;
	 std::vector<COORD> ObjectPos;
    int Size, NumBottles, NumObjects, NumShelves, NumCrates, NumObjectTypes;
	 int E_OBJCHECK, E_BOTTLECHECK, E_OBJPUSH, E_BOTTLEPUSH, E_BPUSHNORTH, E_BPUSHSOUTH, E_BPUSHEAST, E_BPUSHWEST, E_PUSHNORTH, E_PUSHSOUTH, E_PUSHEAST, E_PUSHWEST, E_SAMPLE; //Markers for the beginning of action group
    COORD StartPos;
    double HalfEfficiencyDistance;
    double SmartMoveProb;
    int UncertaintyCount;
	double BIN_ENTROPY_LIMIT; //0.5 or set to preference
	double ACTIVATION_THRESHOLD;
    double PGSAlpha;

private:

    mutable MEMORY_POOL<CELLAR_STATE> MemoryPool;
};

#endif
