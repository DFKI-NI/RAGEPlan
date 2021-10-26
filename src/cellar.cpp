#include "cellar.h"
#include "utils.h"

#define FTABLE_MIN_Q 0.75

using namespace std;
using namespace UTILS;

void CELLAR_STATE::activateFeature(int feature, bool status){
	if(feature >= 0 && feature < Objects.size())
		Objects[feature].active = status;
}

/* Build Ftable mapping features to their afforded actions
 */
void CELLAR::initializeFTable(FTABLE& ftable) const{
	
//	cout << "Creating FTable for Cellar" << endl;

	//Add all 'check' actions
	for(int action=E_OBJCHECK, feature = 0; feature < NumObjects; action++, feature++){		
		ftable.addEntry(action, feature);		
		//cout << "Pushed: " << f_entry << endl;
	}
	
	//Add all 'push' actions
	int feature = 0; //object number
	for(int action=E_PUSHNORTH; action < NumActions; action++){		
		ftable.addEntry(action, feature);
		//cout << "Pushed: " << f_entry << endl;
		
		feature++;
		feature = feature % NumObjects; //Reset to initial object when count reaches NumObjects (ie. new Action)
	}

	ftable.setACTIVATION_THRESHOLD(CELLAR::ACTIVATION_THRESHOLD);
	ftable.setNumFeatures(NumObjects);
	ftable.setNumActions(NumActions);
        
//	cout << "Done" << endl;
}

CELLAR::CELLAR(PROBLEM_PARAMS& problem_params)
{
    CELLAR_PARAMS params = safe_cast<CELLAR_PARAMS&>(problem_params);

    Size = params.size;
    NumBottles = params.bottles;
    NumShelves = params.shelves;
    NumCrates = params.crates;

    Grid = GRID<int>(Size, Size);
    NumObjects = NumShelves + NumCrates;
    NumObjectTypes = 2;
    NumActions = 4; // Movement actions
    NumActions += 2*(NumBottles + NumObjects); //All samples + checks
    NumActions += 4*(NumBottles + NumObjects); //All Pushes

    E_SAMPLE = 4; //First sample action
    E_BOTTLECHECK = E_SAMPLE + NumBottles + NumObjects; //First bottle check
    E_OBJCHECK = E_BOTTLECHECK + NumBottles; //First object check
    //E_BOTTLEPUSH = E_OBJCHECK + NumObjects;
    E_BPUSHNORTH = E_OBJCHECK + NumObjects;
    E_BPUSHSOUTH = E_BPUSHNORTH + NumBottles;
    E_BPUSHEAST = E_BPUSHSOUTH + NumBottles;
    E_BPUSHWEST = E_BPUSHEAST + NumBottles;

    //All object pushes in order
    //E_OBJPUSH = E_BPUSHWEST + NumBottles;
    E_PUSHNORTH = E_BPUSHWEST + NumBottles;
    E_PUSHSOUTH = E_PUSHNORTH + NumObjects;
    E_PUSHEAST = E_PUSHSOUTH + NumObjects;
    E_PUSHWEST = E_PUSHEAST + NumObjects;

    //cout << "Num Actions: " << NumActions << endl;
    //cout << E_SAMPLE << ", " << E_BOTTLECHECK << ", " << E_OBJCHECK << ", " << E_PUSHNORTH << ", " << E_PUSHSOUTH << ", " << E_PUSHEAST << ", " << E_PUSHWEST << endl;

    NumObservations = 3 + NumObjectTypes; //none,good,bad + each object type
    RewardRange = 50; //NOTE: exploration rate recommended to be H_max - H_min, max with c = 0, min with rollouts

    Discount = params.discount;
    fDiscount = params.fDiscount;
    BIN_ENTROPY_LIMIT = params.entropy;
    ACTIVATION_THRESHOLD = params.activation;
    PGSAlpha = params.PGSAlpha;

//    cout << "Gammas: " << Discount << ", " << fDiscount << endl;
//    cout << "Limits: " << BIN_ENTROPY_LIMIT << ", " << ACTIVATION_THRESHOLD << endl;

    RandomSeed(0);

    if (Size == 7 && NumBottles == 8 && NumShelves == 7 && NumCrates == 8)
        Init_7_8();
    else if (Size == 11 && NumBottles == 11 && NumShelves == 15 && NumCrates == 15)
        Init_11_11();
    else if (Size == 5 && NumBottles == 1 && NumShelves == 0 && NumCrates == 4)
        Init_5_1();
    else if (Size == 7 && NumBottles == 2 && NumShelves == 6 && NumCrates == 4)
        Init_Ftest2();
    else if (Size == 5 && NumBottles == 2 && NumShelves == 6 && NumCrates == 4) //Test case for ftable
        Init_Ftest();
    else
        InitGeneral();
}

void CELLAR::Init_5_1(){
	cout << "Using special layout for cellar(5, 1)" << endl;
	
	COORD bottles[] =
	{
	  COORD(2, 2)	  
	};
	
	COORD objects[] =
	{ 
		//Crates
		COORD(1, 2),
		COORD(2, 1),
		COORD(2, 3),
		COORD(3, 2)	
    };
	 
	HalfEfficiencyDistance = 20;
	StartPos = COORD(0, 2);
	Grid.SetAllValues(-1);
	for (int i = 0; i < NumBottles; ++i)
	{
		Grid(bottles[i]) = i;
		BottlePos.push_back(bottles[i]);
	}
	for (int i = 0; i < NumObjects; ++i)
	{      
		if(i < NumShelves)
			Grid(objects[i]) = 100+E_SHELF;
		else
			Grid(objects[i]) = 100+E_CRATE;
		ObjectPos.push_back(objects[i]);
	}
}

void CELLAR::Init_Ftest(){
	cout << "Using special layout for cellar[5, 2, 6, 4] -- Ftable test" << endl;
	
	COORD bottles[] =
	{		
		COORD(1, 4),
		COORD(2, 0)
	};
	
	COORD objects[] =
	{ 
		//Shelves		
		COORD(0, 4),
		COORD(1, 0),
		COORD(2, 4),
		COORD(3, 0),
		COORD(3, 4),
		COORD(4, 0),
		
		//Crates
		COORD(0, 1),
		COORD(1, 3),
		COORD(2, 1),
		COORD(4, 3)	
    };
	 
	HalfEfficiencyDistance = 20;
	StartPos = COORD(0, 2);
	Grid.SetAllValues(-1);
	
	for (int i = 0; i < NumBottles; ++i)
	{
		Grid(bottles[i]) = i;
		BottlePos.push_back(bottles[i]);
	}
	
	for (int i = 0; i < NumObjects; ++i)
	{      
		if(i < NumShelves)
			Grid(objects[i]) = 100+E_SHELF;
		else
			Grid(objects[i]) = 100+E_CRATE;
		ObjectPos.push_back(objects[i]);
	}
}

void CELLAR::Init_Ftest2(){
	cout << "Using special layout for cellar[7, 2, 6, 4] -- Ftable test 2" << endl;
	
	COORD bottles[] =
	{		
		COORD(1, 5),
		COORD(2, 1)
	};
	
	COORD objects[] =
	{ 
		//Shelves		
		COORD(0, 5),
		COORD(1, 1),
		COORD(2, 5),
		COORD(3, 1),
		COORD(3, 5),
		COORD(4, 1),
		
		//Crates
		COORD(0, 2),
		COORD(1, 4),
		COORD(2, 2),
		COORD(4, 4)	
    };
	 
	HalfEfficiencyDistance = 20;
	StartPos = COORD(0, 3);
	Grid.SetAllValues(-1);
	
	for (int i = 0; i < NumBottles; ++i)
	{
		Grid(bottles[i]) = i;
		BottlePos.push_back(bottles[i]);
	}
	
	for (int i = 0; i < NumObjects; ++i)
	{      
		if(i < NumShelves)
			Grid(objects[i]) = 100+E_SHELF;
		else
			Grid(objects[i]) = 100+E_CRATE;
		ObjectPos.push_back(objects[i]);
	}
}

void CELLAR::Init_7_8(){
	cout << "Using special layout similar to rocksample(7, 8)" << endl;
	
	COORD bottles[] =
	{
	  COORD(0, 4),
	  COORD(1, 1),
	  COORD(1, 6),
	  COORD(3, 0),
	  COORD(3, 6),
	  COORD(5, 0),
	  COORD(5, 6),
	  COORD(6, 5)
	};
	
	COORD objects[] =
	{
		//Shelves
		COORD(0, 3),
      COORD(0, 5),
      COORD(1, 0),
		COORD(2, 6),
		COORD(4, 0),
		COORD(4, 6),
		COORD(6, 0),
		 
		//Crates
		COORD(0, 1),
		COORD(1, 4),
		COORD(2, 1),
		COORD(3, 1),
		COORD(3, 5),
		COORD(4, 3),
		COORD(5, 1),
		COORD(5, 4)
    };
	 
	HalfEfficiencyDistance = 20;
	StartPos = COORD(0, 2);
	Grid.SetAllValues(-1);
	for (int i = 0; i < NumBottles; ++i)
	{
		Grid(bottles[i]) = i;
		BottlePos.push_back(bottles[i]);
	}
	for (int i = 0; i < NumObjects; ++i)
	{      
		if(i < NumShelves)
			Grid(objects[i]) = 100+E_SHELF;
		else
			Grid(objects[i]) = 100+E_CRATE;
		ObjectPos.push_back(objects[i]);
	}
}

void CELLAR::Init_11_11(){
	// Equivalent to RockSample_11_11.pomdp(x)
    cout << "Using special layout similar to rocksample(11, 11)" << endl;

	 /*NumShelves = 15;
	 NumCrates = 15;
	 NumObjects = NumShelves + NumCrates;*/
	
    COORD bottles[] =
    {
        COORD(0, 3),
        COORD(0, 7),
        COORD(1, 8),
        COORD(2, 4),
        COORD(3, 3),
        COORD(3, 8),
        COORD(4, 3),
        COORD(5, 8),
        COORD(6, 1),
        COORD(9, 3),
        COORD(9, 9)
    };
	 
	 COORD objects[] =
    {
		 //Shelves
		  COORD(0, 0),
        COORD(0, 10),
        COORD(1, 0),
		  COORD(1, 10),
		  COORD(2, 0),
		  COORD(2, 6),
 		  COORD(2, 10),        
        COORD(3, 6),        
        COORD(4, 5),
		  COORD(4, 6),
		  COORD(4, 7),
		  COORD(10, 0),
		  COORD(10, 1),
		  COORD(10, 2),
		  COORD(10, 3),
		  
		 
		 //Crates
		  COORD(0, 8),
		  COORD(1, 3),
		  COORD(1, 5),
		  COORD(1, 7),		  
		  COORD(4, 2),
		  COORD(4, 8),
		  COORD(5, 1),
		  COORD(5, 2),
		  COORD(5, 5),
		  COORD(6, 5),
		  COORD(7, 5),
		  COORD(7, 8),
		  COORD(8, 2),
		  COORD(8, 8),
		  COORD(9, 7)
    };

    HalfEfficiencyDistance = 20;
    StartPos = COORD(0, 5);
    Grid.SetAllValues(-1);
    for (int i = 0; i < NumBottles; ++i)
    {
        Grid(bottles[i]) = i;
        BottlePos.push_back(bottles[i]);
    }
	 for (int i = 0; i < NumObjects; ++i)
    {      
      if(i < NumShelves)
			Grid(objects[i]) = 100+E_SHELF;
		else
			Grid(objects[i]) = 100+E_CRATE;
		ObjectPos.push_back(objects[i]);
    }
}

/*
	The Grid exists for consistency across start states.  This way multiple (starting) instances of the same problem (ie. object location)
can be created.  This is used when creating state samples for belief state.
*/
void CELLAR::InitGeneral()
{
    HalfEfficiencyDistance = 20;
    StartPos = COORD(0, Size / 2);
    RandomSeed(0);
    Grid.SetAllValues(-1);
	//Place Bottles
    for (int i = 0; i < NumBottles; ++i)
    {
        COORD pos;
        do
        {
            pos = COORD(Random(Size), Random(Size));
        }
        while (Grid(pos) >= 0);
        Grid(pos) = i;
        BottlePos.push_back(pos);
    }
	 	 
	 //Place Objects
	 
	 for (int i = 0; i < NumObjects; ++i)
    {
      COORD pos;
      do
      {
          pos = COORD(Random(Size), Random(Size));
      }
      while (Grid(pos) != -1 || pos == StartPos);
      if(i < NumShelves)
			Grid(pos) = 100+E_SHELF;
		else
			Grid(pos) = 100+E_CRATE;
		ObjectPos.push_back(pos);
    }
}

STATE* CELLAR::Copy(const STATE& state) const
{
    const CELLAR_STATE& cellarstate = safe_cast<const CELLAR_STATE&>(state);
    CELLAR_STATE* newstate = MemoryPool.Allocate();
    *newstate = cellarstate;
    return newstate;
}

void CELLAR::Validate(const STATE& state) const
{
    const CELLAR_STATE& cellarstate = safe_cast<const CELLAR_STATE&>(state);
    assert(Grid.Inside(cellarstate.AgentPos));
}

STATE* CELLAR::CreateStartState() const
{
    CELLAR_STATE* cellarstate = MemoryPool.Allocate();
    cellarstate->AgentPos = StartPos;
	cellarstate->CollectedBottles = 0;
    cellarstate->Bottles.clear();
	cellarstate->Objects.clear();
	
    for (int i = 0; i < NumBottles; i++)
    {
        CELLAR_STATE::ENTRY entry;
        entry.Collected = false;
		  if(NumBottles <= 2){
			  //For the special cases with 1 and 2 bottles, make sure the first one is always good and the second one bad
			entry.Valuable = 1; //(i == 0)? 1 : 0;				
		  }
		  else
			entry.Valuable = Bernoulli(0.5);
		  
        entry.Count = 0;
        entry.Measured = 0;
        entry.ProbValuable = 0.5;
        entry.LikelihoodValuable = 1.0;
        entry.LikelihoodWorthless = 1.0;
        cellarstate->Bottles.push_back(entry);
    }
	 
	for (int i = 0; i < NumObjects; i++)
    {
        CELLAR_STATE::OBJ_ENTRY entry;
        entry.Type = Grid(ObjectPos[i])-100; //types[i];
		entry.ObjPos = ObjectPos[i];
        entry.Count = 0;
        entry.Measured = 0;
        entry.ProbCrate = 0.5;
        entry.LikelihoodCrate = 1.0;
        entry.LikelihoodShelf = 1.0;
		entry.AssumedType = E_NONE;
		entry.active = true;
        cellarstate->Objects.push_back(entry);
    }
	
    assert(cellarstate->Objects.size() == NumObjects);
     
	return cellarstate;
}

void CELLAR::FreeState(STATE* state) const
{
    CELLAR_STATE* cellarstate = safe_cast<CELLAR_STATE*>(state);
    MemoryPool.Free(cellarstate);
}

bool CELLAR::Step(STATE& state, int action,
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
bool CELLAR::StepPGS(STATE& state, int action,
    int& observation, double& reward) const
{
	double r = 0.0;
	double r2 = 0.0;
	STATE* oldstate = Copy(state);	
	
	bool result = StepNormal(state, action, observation, reward);

	if(reward != -100){//Not terminal or out of bounds
		r2 = PGS(*oldstate);
		r = PGS_RO(*oldstate, state, action, r2);
	
		reward += PGSAlpha*r - PGSAlpha*r2;
	}
	FreeState(oldstate);
	
	return result;
}

/*
 * Regular step function.  Simulates transition from state with action and returns observation and reward
 * Note: added punishment for running into objects
*/
bool CELLAR::StepNormal(STATE& state, int action,
    int& observation, double& reward) const
{
    CELLAR_STATE& cellarstate = safe_cast<CELLAR_STATE&>(state);
    reward = 0;
    observation = E_NONE;
	
	double reward_check = -0.5;
	int reward_move = -1;
	int reward_push = -2;
	
	int reward_sample = 10;
	int reward_terminal = 10;
	int punishment = -10;
	int outOfBounds = -100;
	
    if (action < E_SAMPLE) // move
    {
		COORD pos;
		reward = reward_move; //NOTE: movement punishment

		switch (action)
        {
            case COORD::E_EAST:
                pos = COORD(cellarstate.AgentPos.X+1, cellarstate.AgentPos.Y);
                if (cellarstate.AgentPos.X + 1 < Size){
				    if(FreeTile(cellarstate, pos)) cellarstate.AgentPos.X++;
					else reward = punishment;
                    break;
                }
                else{// Maze exit
                    if(cellarstate.CollectedBottles >= 1){
                        reward = reward_terminal;                        
                        return true;
                    }
                    break;
                }
					 
            case COORD::E_NORTH:
                pos = COORD(cellarstate.AgentPos.X, cellarstate.AgentPos.Y+1);
                if (cellarstate.AgentPos.Y + 1 < Size){
				    if(FreeTile(cellarstate, pos)) cellarstate.AgentPos.Y++;
				    else reward = punishment;
				}
                else
                    reward = outOfBounds;
                break;

            case COORD::E_SOUTH:
				pos = COORD(cellarstate.AgentPos.X, cellarstate.AgentPos.Y-1);
                if (cellarstate.AgentPos.Y - 1 >= 0){
				    if(FreeTile(cellarstate, pos)) cellarstate.AgentPos.Y--;
				    else reward = punishment;
				}
                else
                    reward = outOfBounds;
                break;

            case COORD::E_WEST:
				pos = COORD(cellarstate.AgentPos.X-1, cellarstate.AgentPos.Y);
                if (cellarstate.AgentPos.X - 1 >= 0){
				    if(FreeTile(cellarstate, pos)) cellarstate.AgentPos.X--;
					else reward = punishment;
                }
                else
                    reward = outOfBounds;
                break;
        }
    }

    if (action >= E_SAMPLE && action < E_BOTTLECHECK) // sample
    {
        int bottle = Grid(cellarstate.AgentPos);
        if (bottle >= 0 && bottle < NumBottles && !cellarstate.Bottles[bottle].Collected)
        {
            cellarstate.Bottles[bottle].Collected = true;
            if (cellarstate.Bottles[bottle].Valuable){
					reward = reward_sample;
					cellarstate.CollectedBottles++;
				}
            else
                reward = punishment;
        }
        else
        {
            reward = outOfBounds;
        }
    }
	 
	 /* Push actions:
			- Crates cannot be pushed outside of the grid
			- Shelves cannot budge and yield -10 reward
			- Pushing a crate moves it in the desired direction unless it's blocked by another object				
	 */
	if(action >= E_BPUSHNORTH){
		COORD pos1;
		COORD pos2;
		int offsetX = 0;
		int offsetY = 0;
		
		int actionRange = A_PUSHNORTH;
		if(action < E_PUSHNORTH){ //if pushing bottles
			if(action >= E_BPUSHWEST) actionRange = A_PUSHWEST;
			else if(action >= E_BPUSHEAST) actionRange = A_PUSHEAST;
			else if(action >= E_BPUSHSOUTH) actionRange = A_PUSHSOUTH;
		}
		else if(action >= E_PUSHWEST) actionRange = A_PUSHWEST;
		else if(action >= E_PUSHEAST) actionRange = A_PUSHEAST;
		else if(action >= E_PUSHSOUTH) actionRange = A_PUSHSOUTH;
						
		switch(actionRange){
			case A_PUSHNORTH:
				pos1 = COORD(cellarstate.AgentPos.X, cellarstate.AgentPos.Y+1);
				pos2 = COORD(cellarstate.AgentPos.X, cellarstate.AgentPos.Y+2);
				offsetX = 0;
				offsetY = 1;
				break;
			case A_PUSHSOUTH:
				pos1 = COORD(cellarstate.AgentPos.X, cellarstate.AgentPos.Y-1);
				pos2 = COORD(cellarstate.AgentPos.X, cellarstate.AgentPos.Y-2);
				offsetX = 0;
				offsetY = -1;
				break;
			case A_PUSHWEST:
				pos1 = COORD(cellarstate.AgentPos.X-1, cellarstate.AgentPos.Y);
				pos2 = COORD(cellarstate.AgentPos.X-2, cellarstate.AgentPos.Y);
				offsetX = -1;
				offsetY = 0;
				break;
			case A_PUSHEAST:
				pos1 = COORD(cellarstate.AgentPos.X+1, cellarstate.AgentPos.Y);
				pos2 = COORD(cellarstate.AgentPos.X+2, cellarstate.AgentPos.Y);
				offsetX = 1;
				offsetY = 0;
				break;
		}
	
	    /* Push crate and move agent IF: 
		 - there is a crate in the requested direction
		 - the crate will end up inside the grid
		 - the crate will end up on an empty cell
		*/
		if (Grid.Inside(pos1)){
			if(CrateAt(cellarstate, pos1) && Grid.Inside(pos2) && EmptyTile(cellarstate, pos2)){
				int objNum = ObjectNumber(cellarstate, pos1);				
								
				// Update the agent
				cellarstate.AgentPos.X += offsetX;
				cellarstate.AgentPos.Y += offsetY;
				
				// Update the objects known position
				cellarstate.Objects[objNum].ObjPos.X += offsetX;
				cellarstate.Objects[objNum].ObjPos.Y += offsetY;
				
				reward = reward_push; //Push cost punishment
			}
			else
				reward = punishment;
			//TODO: pushing breaks bottles? (ie not valuable)
		}
	}

	if (action >= E_BOTTLECHECK && action < E_OBJCHECK) // Bottle check
    {
        int bottle = action - E_BOTTLECHECK;
        assert(bottle < NumBottles);
        observation = GetObservation(cellarstate, bottle, 1);
        cellarstate.Bottles[bottle].Measured++;

        double distance = COORD::EuclideanDistance(cellarstate.AgentPos, BottlePos[bottle]);
    	double efficiency = (1 + pow(2, -distance / HalfEfficiencyDistance)) * 0.5;

        if (observation == E_GOOD)
        {
            cellarstate.Bottles[bottle].Count++;
            cellarstate.Bottles[bottle].LikelihoodValuable *= efficiency;
            cellarstate.Bottles[bottle].LikelihoodWorthless *= 1.0 - efficiency;

        }
        else
        {
            cellarstate.Bottles[bottle].Count--;
            cellarstate.Bottles[bottle].LikelihoodWorthless *= efficiency;
            cellarstate.Bottles[bottle].LikelihoodValuable *= 1.0 - efficiency;
		  }
		double denom = (0.5 * cellarstate.Bottles[bottle].LikelihoodValuable) +
			(0.5 * cellarstate.Bottles[bottle].LikelihoodWorthless);
		cellarstate.Bottles[bottle].ProbValuable = (0.5 * cellarstate.Bottles[bottle].LikelihoodValuable) / denom;
		  
		//NOTE: Check action punishment
		reward = reward_check;
    }
	 
	 // Check object returns a noisy reading of the type
	 // Approx. probability of object being a crate
	if (action >= E_OBJCHECK && action < E_BPUSHNORTH) // Object check
    {
        int obj = action - E_OBJCHECK;
        assert(obj < NumObjects);
		 
        observation = GetObservation(cellarstate, obj, 2);
        cellarstate.Objects[obj].Measured++;

        double distance = COORD::EuclideanDistance(cellarstate.AgentPos, cellarstate.Objects[obj].ObjPos);
    	double efficiency = (1 + pow(2, -distance / HalfEfficiencyDistance)) * 0.5;

        if (observation == E_CRATE)
        {
            cellarstate.Objects[obj].Count++;
            cellarstate.Objects[obj].LikelihoodCrate *= efficiency;
            cellarstate.Objects[obj].LikelihoodShelf *= 1.0 - efficiency;

        }
        else
        {
            cellarstate.Objects[obj].Count--;
            cellarstate.Objects[obj].LikelihoodShelf *= efficiency;
            cellarstate.Objects[obj].LikelihoodCrate *= 1.0 - efficiency;
		}
		double denom = (0.5 * cellarstate.Objects[obj].LikelihoodCrate) +
			(0.5 * cellarstate.Objects[obj].LikelihoodShelf);
		cellarstate.Objects[obj].ProbCrate = (0.5 * cellarstate.Objects[obj].LikelihoodCrate) / denom;
		
		//As soon as entropy is reduced, assume the closest type
		if(cellarstate.Objects[obj].AssumedType == E_NONE){
			double p = cellarstate.Objects[obj].ProbCrate;
			double binaryEntropy = -1*p*log2(p) - (1-p)*log2(1-p);
		
			if(binaryEntropy <= CELLAR::BIN_ENTROPY_LIMIT){
				//cout << "For object " << obj << " ";
				if(round(cellarstate.Objects[obj].ProbCrate)){
					cellarstate.Objects[obj].AssumedType = E_CRATE;
					//cout << "with P(crate) = " << cellarstate.Objects[obj].ProbCrate << " assumed CRATE" << endl;
				}
				else{
					cellarstate.Objects[obj].AssumedType = E_SHELF;
					//cout << "with P(crate) = " << cellarstate.Objects[obj].ProbCrate << " assumed SHELF" << endl;
				}
			}
		}
		//Note: Check action punishment
		reward = reward_check;
    }
	 	 
    return false;
}


//Create local domain transformations
bool CELLAR::LocalMove(STATE& state, const HISTORY& history,
    int stepObs, const STATUS& status) const
{
    CELLAR_STATE& cellarstate = safe_cast<CELLAR_STATE&>(state);
    int bottle, obj;
	
    //Modify a random bottle
	bottle = Random(NumBottles);
	cellarstate.Bottles[bottle].Valuable = !cellarstate.Bottles[bottle].Valuable;	

	//Modify a random object
	obj = Random(NumObjects);
	if(cellarstate.Objects[obj].Type == E_CRATE)
		cellarstate.Objects[obj].Type = E_SHELF;
	else
		cellarstate.Objects[obj].Type = E_CRATE;
	 
    if (history.Back().Action >= E_BOTTLECHECK){
        //Bottle check?
		if(history.Back().Action < E_OBJCHECK){
            bottle = history.Back().Action - E_BOTTLECHECK;
            int realObs = history.Back().Observation;

            // Condition new state on real observation
            int newObs = GetObservation(cellarstate, bottle, 1);
            if (newObs != realObs)
                return false;

            // Update counts to be consistent with real observation
            if (realObs == E_GOOD && stepObs == E_BAD)
                cellarstate.Bottles[bottle].Count += 2;
            if (realObs == E_BAD && stepObs == E_GOOD)
                cellarstate.Bottles[bottle].Count -= 2;		  
        }
        else{ //Object check
            //Compare with last observation for consistency with history
            obj = history.Back().Action - E_OBJCHECK;
            int realObs = history.Back().Observation;

            // Condition new state on real observation
            int newObs = GetObservation(cellarstate, obj, 2);
            if (newObs != realObs)
                return false;

            // Update counts to be consistent with real observation
            if (realObs == E_CRATE && stepObs == E_SHELF)
                cellarstate.Objects[obj].Count += 2;
            if (realObs == E_SHELF && stepObs == E_CRATE)
                cellarstate.Objects[obj].Count -= 2;	
        }
    }
	 
    return true;
}

/* PGS Rollout policy
 * Simplified PGS point count
 */
double CELLAR::PGS_RO(STATE& oldstate, STATE& state, int action, double oldpgs) const
{
	double points = 0.0;
	double oldpoints = 0.0;
	
	//1. Cast to cellarstate
	CELLAR_STATE& cellarstate = safe_cast<CELLAR_STATE&>(state);
	CELLAR_STATE& oldcellarstate = safe_cast<CELLAR_STATE&>(oldstate);
	
	int bottle = -1;
	if(action >= E_SAMPLE && action < E_BOTTLECHECK){
		bottle = Grid(cellarstate.AgentPos);
		if (cellarstate.Bottles[bottle].Valuable){
			if(cellarstate.Bottles[bottle].Count)
				points++; //+1 for sampling rocks w/ good observations
		}
		else points--;
		
	}
	else if (action >= E_BOTTLECHECK && action < E_OBJCHECK){ //Bottle check
		bottle = action - E_BOTTLECHECK;
		double p = cellarstate.Bottles[bottle].ProbValuable;
		double binaryEntropy = -1*p*log2(p) - (1-p)*log2(1-p);
		if(binaryEntropy > CELLAR::BIN_ENTROPY_LIMIT) points--;
		
		p = oldcellarstate.Bottles[bottle].ProbValuable;
		double oldBinaryEntropy = -1*p*log2(p) - (1-p)*log2(1-p);
		if(oldBinaryEntropy > CELLAR::BIN_ENTROPY_LIMIT) oldpoints--;
		//else points -= 0.5;
	}

	//Update difference for current bottle
	double result = oldpgs - oldpoints + points;

	return result;
}


/*
 * PGS point count
 * */
double CELLAR::PGS(STATE& state) const
{
	double points = 0.0;
	double heuristicProbLimit = 0.4;
	
	//1. Cast to cellarstate
	CELLAR_STATE& cellarstate = safe_cast<CELLAR_STATE&>(state);
	
	//2. Sample	
	for(int bottle=0; bottle < NumBottles; ++bottle){
		if(cellarstate.Bottles[bottle].Collected){
			if (cellarstate.Bottles[bottle].Valuable){
				if(cellarstate.Bottles[bottle].Count)
					points++; //+1 for sampling rocks w/ good observations
			}
			else points--;
		}
		else{
			double p = cellarstate.Bottles[bottle].ProbValuable;
			double binaryEntropy = -1*p*log2(p) - (1-p)*log2(1-p);
			if(binaryEntropy > CELLAR::BIN_ENTROPY_LIMIT) points--;

		}
	}
	
	return points;
}

// PGS Rollout policy
// Computes PGS only for non-checking actions
void CELLAR::GeneratePGS(const STATE& state, const HISTORY& history,
    vector<int>& legal, const STATUS& status) const
{
	static vector<int> acts;
	acts.clear();
	STATE * newstate;
	STATE * oldstate = Copy(state);
	PGSLegal(state, history, acts, status);
	int numLegal = acts.size();
	
	double pgs_values[numLegal]; //pgs_values[i] <-- legalMove[i]
	double pgs_state = PGS(*oldstate);	
		
	int max_p = -1;
	double max_v = -Infinity;	
	
	int observation;
	double reward;
	
	//cout << "Generating PGS values..." << endl;
	//cout << "Found " << numLegal << " legal actions." << endl;
		
	for(unsigned int i=0; i<numLegal; i++){		
		newstate = Copy(state);
		
		StepNormal(*newstate, acts[i], observation, reward); //Simulate transition with action a
		
		// Using regular PGS (slow)
		//pgs_values[i] = PGS(*newstate);
		
		// Adding only PGS differences (fast)
		pgs_values[i] = PGS_RO(*oldstate, *newstate, acts[i], pgs_state); //add differences

		FreeState(newstate);		
	}
	
	FreeState(oldstate);
	max_p = std::distance(pgs_values, max_element(pgs_values, pgs_values+numLegal));
	max_v = pgs_values[max_p];
	assert(max_p > -1);
	
	legal.push_back(acts[max_p]); //Add best action to return vector
	// Add other maxima
	for(int i=0; i<numLegal; i++){
		if(i != max_p && pgs_values[i] == max_v)
            legal.push_back(acts[i]);
	}

}

/*
 * PGS actions
 */
void CELLAR::PGSLegal(const STATE& state, const HISTORY& history,
    vector<int>& legal, const STATUS& status) const
{

    const CELLAR_STATE& cellarstate =
        safe_cast<const CELLAR_STATE&>(state);

	COORD pos = COORD(cellarstate.AgentPos.X, cellarstate.AgentPos.Y);
	COORD posN = COORD(cellarstate.AgentPos.X, cellarstate.AgentPos.Y+1);
	COORD posS = COORD(cellarstate.AgentPos.X, cellarstate.AgentPos.Y-1);
	COORD posW = COORD(cellarstate.AgentPos.X-1, cellarstate.AgentPos.Y);
	COORD posE = COORD(cellarstate.AgentPos.X+1, cellarstate.AgentPos.Y);		
	
	//Move is only possible when there are no obstacles
    if (cellarstate.AgentPos.Y + 1 < Size && FreeTile(cellarstate, posN))
        legal.push_back(COORD::E_NORTH);

	 if(FreeTile(cellarstate, posE))
		  legal.push_back(COORD::E_EAST);

    if (cellarstate.AgentPos.Y - 1 >= 0 && FreeTile(cellarstate, posS))
        legal.push_back(COORD::E_SOUTH);

    if (cellarstate.AgentPos.X - 1 >= 0 && FreeTile(cellarstate, posW))
        legal.push_back(COORD::E_WEST);

	// If standing over a non sampled bottle, allow sample
    int bottle = Grid(cellarstate.AgentPos);
    if (bottle >= 0 && bottle < NumBottles && !cellarstate.Bottles[bottle].Collected){
        legal.push_back(E_SAMPLE + bottle);
	 }

	// 'Check' possible for non sampled bottles
    for (bottle = 0; bottle < NumBottles; ++bottle)
        if (!cellarstate.Bottles[bottle].Collected)
            legal.push_back(E_BOTTLECHECK + bottle);
	
	/* After LNAI paper: arg max_a \in A pgs(s,a)
		where A are uncertainty reducing actions
	*/
	// 'Check' possible for ACTIVE objects without an assumed type		  
	for (int obj = 0; obj < NumObjects; ++obj){
		if(cellarstate.Objects[obj].AssumedType == E_NONE && cellarstate.Objects[obj].active)
			legal.push_back(E_OBJCHECK + obj);
	}
		  
	//Additional actions for adjacent objects
	//Push actions for adjacent objects	 		  		  
	bool objN = false;
	bool objE = false;
	bool objS = false;
    bool objW = false;
	 
	int numObjN, numObjS, numObjE, numObjW;	 

	//Pushing bottles is allowed
	if(Grid.Inside(posN) && Grid(posN) >= 0 && Grid(posN) < NumBottles){
		legal.push_back(E_BPUSHNORTH + Grid(posN));
	}
	if(Grid.Inside(posS) && Grid(posS) >= 0 && Grid(posS) < NumBottles){
		legal.push_back(E_BPUSHSOUTH + Grid(posS));
	}
	if(Grid.Inside(posE) && Grid(posE) >= 0 && Grid(posE) < NumBottles){
		legal.push_back(E_BPUSHEAST + Grid(posE));
	}
	if(Grid.Inside(posW) && Grid(posW) >= 0 && Grid(posW) < NumBottles){
		legal.push_back(E_BPUSHWEST + Grid(posW));
	}	 
	 
	bool objsFound = false; //objN && objE && objS && objW;

	//Pushing is allowed for active objects only
	for(int i=0; i<cellarstate.Objects.size() && !objsFound; i++){
		if(cellarstate.Objects[i].active){ //only use active objects
			if(cellarstate.Objects[i].ObjPos == posN){
				objN = true;
				numObjN = i;
			}
			if(cellarstate.Objects[i].ObjPos == posE){
				objE = true;
				numObjE = i;
			}
			if(cellarstate.Objects[i].ObjPos == posS){
				objS = true;
				numObjS = i;
			}
			if(cellarstate.Objects[i].ObjPos == posW){
				objW = true;
				numObjW = i;
			}
			objsFound = objN && objE && objS && objW;
		}
	}
		  
		  
	if (cellarstate.AgentPos.Y + 1 < Size && objN)
        legal.push_back(E_PUSHNORTH + numObjN);

	if (cellarstate.AgentPos.X + 1 < Size && objE)
		  legal.push_back(E_PUSHEAST + numObjE);

    if (cellarstate.AgentPos.Y - 1 >= 0 && objS)
        legal.push_back(E_PUSHSOUTH + numObjS);

    if (cellarstate.AgentPos.X - 1 >= 0 && objW)
        legal.push_back(E_PUSHWEST + numObjW);
}

/*
    Uniformly random (legal) actions
*/
void CELLAR::GenerateLegal(const STATE& state, const HISTORY& history,
    vector<int>& legal, const STATUS& status) const
{

    const CELLAR_STATE& cellarstate =
        safe_cast<const CELLAR_STATE&>(state);

	 COORD pos = COORD(cellarstate.AgentPos.X, cellarstate.AgentPos.Y);
	 COORD posN = COORD(cellarstate.AgentPos.X, cellarstate.AgentPos.Y+1);
	 COORD posS = COORD(cellarstate.AgentPos.X, cellarstate.AgentPos.Y-1);
	 COORD posW = COORD(cellarstate.AgentPos.X-1, cellarstate.AgentPos.Y);
	 COORD posE = COORD(cellarstate.AgentPos.X+1, cellarstate.AgentPos.Y);		
	
	//Move is only possible when there are no obstacles
    if (cellarstate.AgentPos.Y + 1 < Size && FreeTile(cellarstate, posN))
        legal.push_back(COORD::E_NORTH);

	 if(FreeTile(cellarstate, posE))
		  legal.push_back(COORD::E_EAST);

    if (cellarstate.AgentPos.Y - 1 >= 0 && FreeTile(cellarstate, posS))
        legal.push_back(COORD::E_SOUTH);

    if (cellarstate.AgentPos.X - 1 >= 0 && FreeTile(cellarstate, posW))
        legal.push_back(COORD::E_WEST);

	 // If standing over a non sampled bottle, allow sample
    int bottle = Grid(cellarstate.AgentPos);
    if (bottle >= 0 && bottle < NumBottles && !cellarstate.Bottles[bottle].Collected){
        legal.push_back(E_SAMPLE + bottle);
	 }

	 // 'Check' possible for non sampled bottles
    for (bottle = 0; bottle < NumBottles; ++bottle)
        if (!cellarstate.Bottles[bottle].Collected)
            legal.push_back(E_BOTTLECHECK + bottle);
	
	 // 'Check' possible for all other objects, always
	 for (int obj = 0; obj < NumObjects; ++obj)
      legal.push_back(E_OBJCHECK + obj);
		  
	//Push actions for adjacent objects	 		  		  
	bool objN = false; //(Grid.Inside(posN) && Grid(posN) >= 0)? Grid(posN) < NumBottles : false;
	bool objE = false; //(Grid.Inside(posE) && Grid(posE) >= 0)? Grid(posE) < NumBottles : false;
	bool objS = false; //(Grid.Inside(posS) && Grid(posS) >= 0)? Grid(posS) < NumBottles : false;
    bool objW = false; //(Grid.Inside(posW) && Grid(posW) >= 0)? Grid(posW) < NumBottles : false;
	 
	if(Grid.Inside(posN) && Grid(posN) >= 0 && Grid(posN) < NumBottles){
		legal.push_back(E_BPUSHNORTH + Grid(posN));
	}
	if(Grid.Inside(posS) && Grid(posS) >= 0 && Grid(posS) < NumBottles){
		legal.push_back(E_BPUSHSOUTH + Grid(posS));
	}
	if(Grid.Inside(posE) && Grid(posE) >= 0 && Grid(posE) < NumBottles){
		legal.push_back(E_BPUSHEAST + Grid(posE));
	}
	if(Grid.Inside(posW) && Grid(posW) >= 0 && Grid(posW) < NumBottles){
		legal.push_back(E_BPUSHWEST + Grid(posW));
	}	 
	 
		  
	int numObjN, numObjS, numObjE, numObjW;
	 
	bool objsFound = false; //objN && objE && objS && objW;
		  
	for(int i=0; i<cellarstate.Objects.size() && !objsFound; i++){
	 	if(cellarstate.Objects[i].active){
			if (cellarstate.Objects[i].ObjPos == posN) {
				objN = true;
				numObjN = i;
			}
			if (cellarstate.Objects[i].ObjPos == posE) {
				objE = true;
				numObjE = i;
			}
			if (cellarstate.Objects[i].ObjPos == posS) {
				objS = true;
				numObjS = i;
			}
			if (cellarstate.Objects[i].ObjPos == posW) {
				objW = true;
				numObjW = i;
			}
			objsFound = objN && objE && objS && objW;
		}
	}
		  
		  
	if (cellarstate.AgentPos.Y + 1 < Size && objN)
        legal.push_back(E_PUSHNORTH + numObjN);

	if (cellarstate.AgentPos.X + 1 < Size && objE)
		legal.push_back(E_PUSHEAST + numObjE);

    if (cellarstate.AgentPos.Y - 1 >= 0 && objS)
        legal.push_back(E_PUSHSOUTH + numObjS);

    if (cellarstate.AgentPos.X - 1 >= 0 && objW)
        legal.push_back(E_PUSHWEST + numObjW);

}

/*
  Preferred actions RO policy based on Rocksample.
*/
void CELLAR::GeneratePreferred(const STATE& state, const HISTORY& history,
    vector<int>& actions, const STATUS& status) const
{
		static const bool UseBlindPolicy = false;

		if (UseBlindPolicy)
		{
			actions.push_back(COORD::E_EAST);
			return;
		}

		const CELLAR_STATE& cellarstate =
				  safe_cast<const CELLAR_STATE&>(state);

		// Sample rocks with more +ve than -ve observations
		int rock = Grid(cellarstate.AgentPos);
		if (rock >= 0 && rock < NumBottles && !cellarstate.Bottles[rock].Collected)
		{
			int total = 0;
			for (int t = 0; t < history.Size(); ++t)
			{
				if (history[t].Action == rock + 1 + E_SAMPLE)
				{
					if (history[t].Observation == E_GOOD)
						total++;
					if (history[t].Observation == E_BAD)
						total--;
				}
			}
			if (total > 0)
			{
				actions.push_back(E_SAMPLE + rock);
				return;
			}

		}

		// processes the bottles
		bool all_bad = true;
		bool north_interesting = false;
		bool south_interesting = false;
		bool west_interesting  = false;
		bool east_interesting  = false;

		for (int rock = 0; rock < NumBottles; ++rock)
		{
			const CELLAR_STATE::ENTRY& entry = cellarstate.Bottles[rock];
			if (!entry.Collected)
			{
				int total = 0;
				for (int t = 0; t < history.Size(); ++t)
				{
					if (history[t].Action == rock + 1 + E_SAMPLE)
					{
						if (history[t].Observation == E_GOOD)
							total++;
						if (history[t].Observation == E_BAD)
							total--;
					}
				}

				if (total >= 0)
				{
					all_bad = false;

					if (BottlePos[rock].Y > cellarstate.AgentPos.Y)
						north_interesting = true;
					if (BottlePos[rock].Y < cellarstate.AgentPos.Y)
						south_interesting = true;
					if (BottlePos[rock].X < cellarstate.AgentPos.X)
						west_interesting = true;
					if (BottlePos[rock].X > cellarstate.AgentPos.X)
						east_interesting = true;
				}
			}
		}

		// if all remaining rocks seem bad, then head east
		if (all_bad)
		{
			actions.push_back(COORD::E_EAST);
			return;
		}

		// generate a random legal move, with the exceptions that:
		//   a) there is no point measuring a rock that is already collected
		//   b) there is no point measuring a rock too often
		//   c) there is no point measuring a rock which is clearly bad or good
		//   d) we never sample a rock (since we need to be sure)
		//   e) we never move in a direction that doesn't take us closer to
		//      either the edge of the map or an interesting rock
		COORD posN = COORD(cellarstate.AgentPos.X, cellarstate.AgentPos.Y+1);
		COORD posS = COORD(cellarstate.AgentPos.X, cellarstate.AgentPos.Y-1);
		COORD posW = COORD(cellarstate.AgentPos.X-1, cellarstate.AgentPos.Y);
		COORD posE = COORD(cellarstate.AgentPos.X+1, cellarstate.AgentPos.Y);
		
		if (cellarstate.AgentPos.Y + 1 < Size && north_interesting && FreeTile(cellarstate, posN))
				actions.push_back(COORD::E_NORTH);

		if (east_interesting && FreeTile(cellarstate, posE))
			actions.push_back(COORD::E_EAST);

		if (cellarstate.AgentPos.Y - 1 >= 0 && south_interesting && FreeTile(cellarstate, posS))
			actions.push_back(COORD::E_SOUTH);

		if (cellarstate.AgentPos.X - 1 >= 0 && west_interesting && FreeTile(cellarstate, posW))
			actions.push_back(COORD::E_WEST);


		//Bottle checks
		for (rock = 0; rock < NumBottles; ++rock)
		{
			if (!cellarstate.Bottles[rock].Collected    &&
				cellarstate.Bottles[rock].ProbValuable != 0.0 &&
				cellarstate.Bottles[rock].ProbValuable != 1.0 &&
				cellarstate.Bottles[rock].Measured < 5  &&
				std::abs(cellarstate.Bottles[rock].Count) < 2)
			{
				actions.push_back(E_BOTTLECHECK + rock);
			}
		}
		
		//Use similar rules for object checks
		for (int obj = 0; obj < NumObjects; ++obj)
		{
			if (cellarstate.Objects[obj].ProbCrate != 0.0 &&
				cellarstate.Objects[obj].ProbCrate != 1.0 &&
				cellarstate.Objects[obj].Measured < 5  &&
				std::abs(cellarstate.Objects[obj].Count) < 2)
			{
				actions.push_back(E_OBJCHECK + obj);
			}
		}
		
	 //Push actions for adjacent objects
	 	
	bool objN = false; //(Grid.Inside(posN) && Grid(posN) >= 0)? Grid(posN) < NumBottles : false;
	bool objE = false; //(Grid.Inside(posE) && Grid(posE) >= 0)? Grid(posE) < NumBottles : false;
	bool objS = false; //(Grid.Inside(posS) && Grid(posS) >= 0)? Grid(posS) < NumBottles : false;
    bool objW = false; //(Grid.Inside(posW) && Grid(posW) >= 0)? Grid(posW) < NumBottles : false;
	 
	if(Grid.Inside(posN) && Grid(posN) >= 0 && Grid(posN) < NumBottles){
		actions.push_back(E_BPUSHNORTH + Grid(posN));
	}
	if(Grid.Inside(posS) && Grid(posS) >= 0 && Grid(posS) < NumBottles){
		actions.push_back(E_BPUSHSOUTH + Grid(posS));
	}
	if(Grid.Inside(posE) && Grid(posE) >= 0 && Grid(posE) < NumBottles){
		actions.push_back(E_BPUSHEAST + Grid(posE));
	}
	if(Grid.Inside(posW) && Grid(posW) >= 0 && Grid(posW) < NumBottles){
		actions.push_back(E_BPUSHWEST + Grid(posW));
	}
	 
	int numObjN, numObjS, numObjE, numObjW;
	 
	bool objsFound = objN && objE && objS && objW;
		  
	for(int i=0; i<cellarstate.Objects.size() && !objsFound; i++){
        if(cellarstate.Objects[i].active) {
            if (cellarstate.Objects[i].ObjPos == posN) {
                objN = true;
                numObjN = i;
            }
            if (cellarstate.Objects[i].ObjPos == posE) {
                objE = true;
                numObjE = i;
            }
            if (cellarstate.Objects[i].ObjPos == posS) {
                objS = true;
                numObjS = i;
            }
            if (cellarstate.Objects[i].ObjPos == posW) {
                objW = true;
                numObjW = i;
            }
        }
        objsFound = objN && objE && objS && objW;
	}
		  
		  
    if (cellarstate.AgentPos.Y + 1 < Size && objN)
        actions.push_back(E_PUSHNORTH + numObjN);

    if (cellarstate.AgentPos.X + 1 < Size && objE)
        actions.push_back(E_PUSHEAST + numObjE);

    if (cellarstate.AgentPos.Y - 1 >= 0 && objS)
        actions.push_back(E_PUSHSOUTH + numObjS);

    if (cellarstate.AgentPos.X - 1 >= 0 && objW)
        actions.push_back(E_PUSHWEST + numObjW);
}

///// Utility/domain fuctions /////
bool CELLAR::CrateAt(const CELLAR_STATE& cellarstate, const COORD& coord) const{
	for(int i=0; i<cellarstate.Objects.size(); i++){
		if(cellarstate.Objects[i].ObjPos == coord && cellarstate.Objects[i].Type == E_CRATE)
			return true;
	}
	return false;
}

bool CELLAR::ShelfAt(const CELLAR_STATE& cellarstate, const COORD& coord) const{
	for(int i=0; i<cellarstate.Objects.size(); i++){
		if(cellarstate.Objects[i].ObjPos == coord && cellarstate.Objects[i].Type == E_SHELF)
			return true;
	}
	return false;	
}

//Empty if there is no crate, shelf or bottle.
//TODO: find alternative to avoid checking all objects for position
bool CELLAR::EmptyTile(const CELLAR_STATE& cellarstate, const COORD& coord) const{
	bool empty = true;
	
	// If there is a bottle, not empty
	if(Grid.Inside(coord) && Grid(coord) >= 0 && Grid(coord) < NumBottles)
		empty = false;
	//If no bottles but objects, not empty
	if(empty)
		for(int i=0; i<cellarstate.Objects.size() && empty; i++){
			if(cellarstate.Objects[i].ObjPos == coord)
				empty = false;
		}
		
	return empty;
}

// Tiles are free if there are no obstacles (bottles are OK)
bool CELLAR::FreeTile(const CELLAR_STATE& cellarstate, const COORD& coord) const{
	return ObjectNumber(cellarstate, coord) < 0;
}

int CELLAR::ObjectNumber(const CELLAR_STATE& cellarstate, const COORD& coord) const{
	for(int i=0; i<cellarstate.Objects.size(); i++){
		if(cellarstate.Objects[i].ObjPos == coord)
			return i;
	}
	return -1;
}
/////

int CELLAR::GetObservation(const CELLAR_STATE& cellarstate, int pos, int type) const
{	 
    double distance;
    double efficiency;
	int obs;
	
    if(type == 1){
        distance = COORD::EuclideanDistance(cellarstate.AgentPos, BottlePos[pos]);
        efficiency = (1 + pow(2, -distance / HalfEfficiencyDistance)) * 0.5;
            
        if (Bernoulli(efficiency))
            obs = cellarstate.Bottles[pos].Valuable ? E_GOOD : E_BAD;
        else
            obs = cellarstate.Bottles[pos].Valuable ? E_BAD : E_GOOD;
    }
    else{
        distance = COORD::EuclideanDistance(cellarstate.AgentPos, cellarstate.Objects[pos].ObjPos);
        efficiency = (1 + pow(2, -distance / HalfEfficiencyDistance)) * 0.5;
            
        if (Bernoulli(efficiency))
            obs = cellarstate.Objects[pos].Type == E_CRATE ? E_CRATE : E_SHELF;
        else
            obs = cellarstate.Objects[pos].Type == E_CRATE ? E_SHELF : E_CRATE;
    }
	
   return obs; 
}

void CELLAR::DisplayBeliefs(const BELIEF_STATE& beliefState,
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

void CELLAR::DisplayState(const STATE& state, std::ostream& ostr) const
{
    const CELLAR_STATE& cellarstate = safe_cast<const CELLAR_STATE&>(state);
    ostr << endl;
    for (int x = 0; x < Size + 2; x++)
        ostr << "# ";
    ostr << endl;
	
	 int thing = 0;
	 int obj = 0;
		
    for (int y = Size - 1; y >= 0; y--)
    {
        ostr << "# ";
        for (int x = 0; x < Size; x++)
        {
            COORD pos(x, y);
            thing = Grid(pos);
			  
            if (cellarstate.AgentPos == COORD(x, y))
                ostr << "* ";
            else
					if (thing >= 0 && thing < NumBottles){ //Display non collected bottles
					    const CELLAR_STATE::ENTRY& entry = cellarstate.Bottles[thing];
                        if(!entry.Collected)
					        ostr << thing << (entry.Valuable ? "$" : "X");
                        else
                            ostr << thing << "X";
					}
					else{
						int obj = ObjectNumber(cellarstate, pos);						
						if(obj >= 0){
							const CELLAR_STATE::OBJ_ENTRY& entry = cellarstate.Objects[obj];
							if(entry.active)
								ostr << obj << (entry.Type == E_SHELF ? "S" : "C");
							else
								ostr << obj << "!";
						}
					   else
						  	ostr << ". ";
					}
        }
        ostr << "#" << endl;
    }
    for (int x = 0; x < Size + 2; x++)
        ostr << "# ";
    ostr << endl;
}

void CELLAR::DisplayObservation(const STATE& state, int observation, std::ostream& ostr) const
{
    switch (observation)
    {
    case E_NONE:
        break;
    case E_GOOD:
        ostr << "Observed good" << endl;
        break;
    case E_BAD:
        ostr << "Observed bad" << endl;
        break;
	 case E_CRATE:
		  ostr << "Observed a crate" << endl;
	     break;
	 case E_SHELF:
		  ostr << "Observed a shelf" << endl;
	     break;
    }
}

void CELLAR::DisplayAction(int action, std::ostream& ostr) const
{
	ostr << "A: ";
    if (action < E_SAMPLE)
        ostr << "Move " << COORD::CompassString[action] << endl;
	 else if (action >= E_PUSHWEST)
        ostr << "Push Obj " << action - E_PUSHWEST << " West" << endl;
	 else if (action >= E_PUSHEAST)
        ostr << "Push Obj " << action - E_PUSHEAST << " East" << endl;
	 else if (action >= E_PUSHSOUTH)
        ostr << "Push Obj " << action - E_PUSHSOUTH << " South" << endl;
	 else if (action >= E_PUSHNORTH)
        ostr << "Push Obj " << action - E_PUSHNORTH << " North" << endl;
    
	 else if (action >= E_BPUSHWEST)
        ostr << "Push Bottle W " << action - E_BPUSHWEST << endl;
	 else if (action >= E_BPUSHEAST)
        ostr << "Push Bottle E " << action - E_BPUSHEAST << endl;
	 else if (action >= E_BPUSHSOUTH)
        ostr << "Push Bottle S " << action - E_BPUSHSOUTH << endl;
	 else if (action >= E_BPUSHNORTH)
        ostr << "Push Bottle N " << action - E_BPUSHNORTH << endl;
	 
	 else if (action >= E_OBJCHECK)
        ostr << "Check object " << action - E_OBJCHECK << endl;
	 else if (action >= E_BOTTLECHECK)
        ostr << "Check bottle " << action - E_BOTTLECHECK << endl;
	 else if (action >= E_SAMPLE)
        ostr << "Sample " << action - E_SAMPLE << endl;
    	 
}
