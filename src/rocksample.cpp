#include "rocksample.h"
#include "utils.h"

#include <omp.h>

using namespace std;
using namespace UTILS;

ROCKSAMPLE::ROCKSAMPLE(int size, int rocks)
:   Grid(size, size),
    Size(size),
    NumRocks(rocks),
    SmartMoveProb(0.95),
    UncertaintyCount(0)
{
    NumActions = NumRocks + 5;
    NumObservations = 3;
    RewardRange = 20;
    Discount = 0.95;
	
	 //RandomSeed(time(NULL));

    if (size == 7 && rocks == 8)
        Init_7_8();
    else if (size == 11 && rocks == 11)
        Init_11_11();
    else
        InitGeneral();
}

void ROCKSAMPLE::InitGeneral()
{
    HalfEfficiencyDistance = 20;
    StartPos = COORD(0, Size / 2);
    RandomSeed(0);
    Grid.SetAllValues(-1);
    for (int i = 0; i < NumRocks; ++i)
    {
        COORD pos;
        do
        {
            pos = COORD(Random(Size), Random(Size));
        }
        while (Grid(pos) >= 0);
        Grid(pos) = i;
        RockPos.push_back(pos);
    }
}

void ROCKSAMPLE::Init_7_8()
{
    // Equivalent to RockSample_7_8.pomdpx
    cout << "Using special layout for rocksample(7, 8)" << endl;

    COORD rocks[] =
    {
        COORD(2, 0),
        COORD(0, 1),
        COORD(3, 1),
        COORD(6, 3),
        COORD(2, 4),
        COORD(3, 4),
        COORD(5, 5),
        COORD(1, 6)
    };

    HalfEfficiencyDistance = 20;
    StartPos = COORD(0, 3);
    Grid.SetAllValues(-1);
    for (int i = 0; i < NumRocks; ++i)
    {
        Grid(rocks[i]) = i;
        RockPos.push_back(rocks[i]);
    }
}

void ROCKSAMPLE::Init_11_11()
{
    // Equivalent to RockSample_11_11.pomdp(x)
    cout << "Using special layout for rocksample(11, 11)" << endl;

    COORD rocks[] =
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

    HalfEfficiencyDistance = 20;
    StartPos = COORD(0, 5);
    Grid.SetAllValues(-1);
    for (int i = 0; i < NumRocks; ++i)
    {
        Grid(rocks[i]) = i;
        RockPos.push_back(rocks[i]);
    }
}


STATE* ROCKSAMPLE::Copy(const STATE& state) const
{
    const ROCKSAMPLE_STATE& rockstate = safe_cast<const ROCKSAMPLE_STATE&>(state);
    ROCKSAMPLE_STATE* newstate = MemoryPool.Allocate();
    *newstate = rockstate;
    return newstate;
}

void ROCKSAMPLE::Validate(const STATE& state) const
{
    const ROCKSAMPLE_STATE& rockstate = safe_cast<const ROCKSAMPLE_STATE&>(state);
    assert(Grid.Inside(rockstate.AgentPos));
}

STATE* ROCKSAMPLE::CreateStartState() const
{
    ROCKSAMPLE_STATE* rockstate = MemoryPool.Allocate();
    rockstate->AgentPos = StartPos;
    rockstate->Rocks.clear();
    for (int i = 0; i < NumRocks; i++)
    {
        ROCKSAMPLE_STATE::ENTRY entry;
        entry.Collected = false;
        entry.Valuable = Bernoulli(0.5);
        entry.Count = 0;
        entry.Measured = 0;
        entry.ProbValuable = 0.5;
        entry.LikelihoodValuable = 1.0;
        entry.LikelihoodWorthless = 1.0;
        rockstate->Rocks.push_back(entry);
    }
    rockstate->Target = SelectTarget(*rockstate);
    return rockstate;
}

void ROCKSAMPLE::FreeState(STATE* state) const
{
    ROCKSAMPLE_STATE* rockstate = safe_cast<ROCKSAMPLE_STATE*>(state);
    MemoryPool.Free(rockstate);
}

bool ROCKSAMPLE::Step(STATE& state, int action,
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

/*** Added by JCS

		Step function with new reward distribution using PGS action biases
***/
bool ROCKSAMPLE::StepPGS(STATE& state, int action,
    int& observation, double& reward) const
{
	double scale = 10.0;
	double r = 0.0;
	double r2 = 0.0;
	STATE* oldstate = Copy(state);	
	
	bool result = StepNormal(state, action, observation, reward);
	 // Potential-based reward bonus
	
	if(reward != -100){//Not terminal or out of bounds
		r2 = PGS(*oldstate);
		r = PGS_RO(*oldstate, state, action, r2);
		
		
		//cout << "reward = " << reward << ", r1 = " << r << ", r2 = " << r2 << endl;
				
		reward += scale*r - scale*r2;
	}
	FreeState(oldstate);
	
	return result;
}

bool ROCKSAMPLE::SimpleStep(STATE& state, int action) const
{
    ROCKSAMPLE_STATE& rockstate = safe_cast<ROCKSAMPLE_STATE&>(state);    
    int observation = E_NONE;

    if (action < E_SAMPLE) // move
    {
        switch (action)
        {
            case COORD::E_EAST:
                if (rockstate.AgentPos.X + 1 < Size)
                {
                    rockstate.AgentPos.X++;
                    break;
                }
                else
                {
                    return true;
                }

            case COORD::E_NORTH:
                if (rockstate.AgentPos.Y + 1 < Size)
                    rockstate.AgentPos.Y++;
                break;

            case COORD::E_SOUTH:
                if (rockstate.AgentPos.Y - 1 >= 0)
                    rockstate.AgentPos.Y--;
                break;

            case COORD::E_WEST:
                if (rockstate.AgentPos.X - 1 >= 0)
                    rockstate.AgentPos.X--;
                break;
        }
    }

    if (action == E_SAMPLE) // sample
    {
        int rock = Grid(rockstate.AgentPos);
        if (rock >= 0 && !rockstate.Rocks[rock].Collected)
        {
            rockstate.Rocks[rock].Collected = true;
        }
    }

    if (action > E_SAMPLE) // check
    {
        int rock = action - E_SAMPLE - 1;
        assert(rock < NumRocks);
        observation = GetObservation(rockstate, rock);
        rockstate.Rocks[rock].Measured++;

        double distance = COORD::EuclideanDistance(rockstate.AgentPos, RockPos[rock]);
    	double efficiency = (1 + pow(2, -distance / HalfEfficiencyDistance)) * 0.5;

        if (observation == E_GOOD)
        {
            rockstate.Rocks[rock].Count++;
            rockstate.Rocks[rock].LikelihoodValuable *= efficiency;
            rockstate.Rocks[rock].LikelihoodWorthless *= 1.0 - efficiency;

        }
        else
        {
            rockstate.Rocks[rock].Count--;
            rockstate.Rocks[rock].LikelihoodWorthless *= efficiency;
            rockstate.Rocks[rock].LikelihoodValuable *= 1.0 - efficiency;
		}
		double denom = (0.5 * rockstate.Rocks[rock].LikelihoodValuable) +
			(0.5 * rockstate.Rocks[rock].LikelihoodWorthless);
		rockstate.Rocks[rock].ProbValuable = (0.5 * rockstate.Rocks[rock].LikelihoodValuable) / denom;
    }

    if (rockstate.Target < 0 || rockstate.AgentPos == RockPos[rockstate.Target])
        rockstate.Target = SelectTarget(rockstate);

    return false;
}

bool ROCKSAMPLE::StepNormal(STATE& state, int action,
    int& observation, double& reward) const
{
    ROCKSAMPLE_STATE& rockstate = safe_cast<ROCKSAMPLE_STATE&>(state);
    reward = 0;
    observation = E_NONE;

    if (action < E_SAMPLE) // move
    {
        switch (action)
        {
            case COORD::E_EAST:
                if (rockstate.AgentPos.X + 1 < Size)
                {
                    rockstate.AgentPos.X++;
                    break;
                }
                else
                {
                    reward = +10;
                    return true;
                }

            case COORD::E_NORTH:
                if (rockstate.AgentPos.Y + 1 < Size)
                    rockstate.AgentPos.Y++;
                else
                    reward = -100;
                break;

            case COORD::E_SOUTH:
                if (rockstate.AgentPos.Y - 1 >= 0)
                    rockstate.AgentPos.Y--;
                else
                    reward = -100;
                break;

            case COORD::E_WEST:
                if (rockstate.AgentPos.X - 1 >= 0)
                    rockstate.AgentPos.X--;
                else
                    reward = -100;
                break;
        }
    }

    if (action == E_SAMPLE) // sample
    {
        int rock = Grid(rockstate.AgentPos);
        if (rock >= 0 && !rockstate.Rocks[rock].Collected)
        {
            rockstate.Rocks[rock].Collected = true;
            if (rockstate.Rocks[rock].Valuable)
                reward = +10;
            else
                reward = -10;
        }
        else
        {
            reward = -100;
        }
    }

    if (action > E_SAMPLE) // check
    {
        int rock = action - E_SAMPLE - 1;
        assert(rock < NumRocks);
        observation = GetObservation(rockstate, rock);
        rockstate.Rocks[rock].Measured++;

        double distance = COORD::EuclideanDistance(rockstate.AgentPos, RockPos[rock]);
    	double efficiency = (1 + pow(2, -distance / HalfEfficiencyDistance)) * 0.5;

        if (observation == E_GOOD)
        {
            rockstate.Rocks[rock].Count++;
            rockstate.Rocks[rock].LikelihoodValuable *= efficiency;
            rockstate.Rocks[rock].LikelihoodWorthless *= 1.0 - efficiency;

        }
        else
        {
            rockstate.Rocks[rock].Count--;
            rockstate.Rocks[rock].LikelihoodWorthless *= efficiency;
            rockstate.Rocks[rock].LikelihoodValuable *= 1.0 - efficiency;
		}
		double denom = (0.5 * rockstate.Rocks[rock].LikelihoodValuable) +
			(0.5 * rockstate.Rocks[rock].LikelihoodWorthless);
		rockstate.Rocks[rock].ProbValuable = (0.5 * rockstate.Rocks[rock].LikelihoodValuable) / denom;
    }

    if (rockstate.Target < 0 || rockstate.AgentPos == RockPos[rockstate.Target])
        rockstate.Target = SelectTarget(rockstate);

    assert(reward != -100);
    return false;
}

bool ROCKSAMPLE::LocalMove(STATE& state, const HISTORY& history,
    int stepObs, const STATUS& status) const
{
    ROCKSAMPLE_STATE& rockstate = safe_cast<ROCKSAMPLE_STATE&>(state);
    int rock = Random(NumRocks);
    rockstate.Rocks[rock].Valuable = !rockstate.Rocks[rock].Valuable;

    if (history.Back().Action > E_SAMPLE) // check rock
    {
        rock = history.Back().Action - E_SAMPLE - 1;
        int realObs = history.Back().Observation;

        // Condition new state on real observation
        int newObs = GetObservation(rockstate, rock);
        if (newObs != realObs)
            return false;

        // Update counts to be consistent with real observation
        if (realObs == E_GOOD && stepObs == E_BAD)
            rockstate.Rocks[rock].Count += 2;
        if (realObs == E_BAD && stepObs == E_GOOD)
            rockstate.Rocks[rock].Count -= 2;
    }
    return true;
}

/*** PGS Rollout policy ***/
double ROCKSAMPLE::PGS_RO(STATE& oldstate, STATE& state, int action, double oldpgs) const
{
	double points = 0.0;
	double oldpoints = 0.0;
	
	//1. Cast to rockstate
	ROCKSAMPLE_STATE& rockstate = safe_cast<ROCKSAMPLE_STATE&>(state);
	ROCKSAMPLE_STATE& oldrockstate = safe_cast<ROCKSAMPLE_STATE&>(oldstate);
	
	int rock = -1;	
	if(action == E_SAMPLE)
		rock = Grid(rockstate.AgentPos);
	else if (action > E_SAMPLE)
		rock = action - E_SAMPLE - 1;
	
	if(rock >= 0){
		//Points for current state
		if(rockstate.Rocks[rock].Collected){
			if (rockstate.Rocks[rock].Valuable){
				if(rockstate.Rocks[rock].Count)
					points++; //+1 for sampling rocks w/ good observations
			}
			else points--;
		}
		else{
			if(rockstate.Rocks[rock].Measured){			
				double p = rockstate.Rocks[rock].ProbValuable;
				double binaryEntropy = -1*p*log2(p) - (1-p)*log2(1-p);			
				if(binaryEntropy > 0.5) points--;						
			}
		}
		
		//Points for previous state
		if(oldrockstate.Rocks[rock].Collected){
			if (oldrockstate.Rocks[rock].Valuable){
				if(oldrockstate.Rocks[rock].Count)
					oldpoints++; //+1 for sampling rocks w/ good observations
			}
			else oldpoints--;
		}
		else{
			if(oldrockstate.Rocks[rock].Measured){			
				double p = oldrockstate.Rocks[rock].ProbValuable;
				double binaryEntropy = -1*p*log2(p) - (1-p)*log2(1-p);			
				if(binaryEntropy > 0.5) oldpoints--;						
			}
		}
	}
	//Update difference for current rock
	double result = oldpgs - oldpoints + points;
	
	/*
	//2. Sample
	if(action == E_SAMPLE){
		int rock = Grid(rockstate.AgentPos);
		double p = oldrockstate.Rocks[rock].ProbValuable;
		double binaryEntropy = -1*p*log2(p) - (1-p)*log2(1-p);
		
		if(rockstate.Rocks[rock].Valuable){
			if(binaryEntropy > 0.5) points++;
			if(rockstate.Rocks[rock].Count) points++;
		}
		else{
			if(binaryEntropy <= 0.5) points--;
		}
	}
	
	//3. Check
	if (action > E_SAMPLE){
		int rock = action - E_SAMPLE - 1;
		
		double p = rockstate.Rocks[rock].ProbValuable;
		double binaryEntropy = -1*p*log2(p) - (1-p)*log2(1-p);
		
		double oldP = oldrockstate.Rocks[rock].ProbValuable;
		double oldBinaryEntropy = -1*oldP*log2(oldP) - (1-oldP)*log2(1-oldP);
		
		if(rockstate.Rocks[rock].Measured == 1){
			if(binaryEntropy > 0.5) points--;
		}
		else{
			if(oldBinaryEntropy > 0.5 && binaryEntropy <= 0.5) points++;
			else if(binaryEntropy > 0.5 && oldBinaryEntropy <= 0.5) points--;
		}
		
		//Works OK:
		//if(binaryEntropy > 0.5 && !oldrockstate.Rocks[rock].Measured || binaryEntropy > 0.5 && oldBinaryEntropy <= 0.5) points--;
		//if(oldBinaryEntropy > 0.5 && binaryEntropy <= 0.5) points++;
	}
	*/
	return result;
}


// PGS score
double ROCKSAMPLE::PGS(STATE& state) const
{
	double points = 0.0;
	double heuristicProbLimit = 0.4;
	
	//1. Cast to rockstate
	ROCKSAMPLE_STATE& rockstate = safe_cast<ROCKSAMPLE_STATE&>(state);
	
	//2. Sample	
	for(int rock=0; rock < NumRocks; ++rock){
		if(rockstate.Rocks[rock].Collected){
			if (rockstate.Rocks[rock].Valuable){
				if(rockstate.Rocks[rock].Count)
					points++; //+1 for sampling rocks w/ good observations
			}
			else points--;
		}
		else{
			//Penalty:
			/*double p = rockstate.Rocks[rock].ProbValuable;
			if(0.1 < p && p < 0.9) points--;
			*/
			if(rockstate.Rocks[rock].Measured){			
				double p = rockstate.Rocks[rock].ProbValuable;
				double binaryEntropy = -1*p*log2(p) - (1-p)*log2(1-p);			
				if(binaryEntropy > 0.5) points--;						
			}
			/*if(rockstate.Rocks[rock].Measured){
				double p = rockstate.Rocks[rock].ProbValuable;
				points += 1 + p*log2(p) + (1-p)*log2(1-p);
			}*/
			/*//double p = rockstate.Rocks[rock].ProbValuable;
			//points += 1 - (-1*p*log2(p) - (1-p)*log2(1-p));
			double nMost = 0;
			double nLeast = 0;
			double measured = rockstate.Rocks[rock].Measured;
			double count = fabs(rockstate.Rocks[rock].Count);
			
			nMost = (measured + count)/2.0; //get No. of observations
			nLeast = measured - nMost;
			
			//double accuracy = count/measured;
			
			double TPR = nMost/measured; //True Positive Rate
			//points += log2(1 + TPR);*/
			
			//Must add points whenever the knowledge about a given rock is more certain.
			
			/*if(rockstate.Rocks[rock].Measured){
				//double diff = fabs(0.5 - rockstate.Rocks[rock].ProbValuable);
				int count = abs(rockstate.Rocks[rock].Count);
				double value = 0.0;
				if(count <= 10) value = 0.25;
				if(count < 5) value = 0.5;
				if(count < 3) value = 1.0;		
				
				//cout << "rock " << rock << " w/ P = " << rockstate.Rocks[rock].ProbValuable << " and diff " << diff <<" worth " << value << " points." << endl;
				points+=value;
			}*/
		}
		/*if(rockstate.Rocks[rock].Measured){
			double nMost = 0;
			double nLeast = 0;
			int measured = rockstate.Rocks[rock].Measured;
			int count = rockstate.Rocks[rock].Count;
			
			nMost = (double) measured/2.0 + fabs(count/2.0); //get No. of observations
			nLeast = measured - nMost;
			
			double ratio = 0.5;
			ratio = nMost / measured;
			
			points += 1 + log2(ratio); //estimate information value of observation
		}*/
		
			/*if(fabs(rockstate.Rocks[rock].LikelihoodValuable - rockstate.Rocks[rock].LikelihoodWorthless) > heuristicProbLimit &&
				rockstate.Rocks[rock].Measured <= 5)
				points++;
				*/
			/*if (rockstate.Rocks[rock].ProbValuable != 0.0 &&
				rockstate.Rocks[rock].ProbValuable != 1.0 &&
				rockstate.Rocks[rock].Measured < 5  &&
				std::abs(rockstate.Rocks[rock].Count) < 2)
				points++;
			*/
			/*if(rockstate.Rocks[rock].Measured >= 5 &&
				(rockstate.Rocks[rock].ProbValuable < heuristicProbLimit ||
				(1.0 - heuristicProbLimit) < rockstate.Rocks[rock].ProbValuable)){
				points++;				
			}*/
			//points += 0.5 + fabs(0.5 - rockstate.Rocks[rock].ProbValuable);
			//points += fabs(rockstate.Rocks[rock].LikelihoodValuable - rockstate.Rocks[rock].LikelihoodWorthless); //Reducing uncertainty?			
			//if(rockstate.Rocks[rock].ProbValuable > 0)
			//	points += rockstate.Rocks[rock].ProbValuable; //Fraction for changes in ProbValuable (will punish checking bad rocks, reward checking good ones)
		//}
	}
	
	//3. Check	

	//if(observation == E_GOOD)
	//	points+=0.5;
	
	
	//Add point for leaving? Prob not because terminal
	
	return points;
}

// PGS Rollout policy
// Computes PGS only for non Checking actions
void ROCKSAMPLE::GeneratePGS(const STATE& state, const HISTORY& history,
    vector<int>& legal, const STATUS& status) const
{
	static vector<int> acts;
	acts.clear();
	STATE * newstate;
	STATE * oldstate = Copy(state);
	GenerateLegal(state, history, acts, status);
	int numLegal = acts.size();
	//STATE * newstate = new STATE*[numLegal];
	//for(int i=0; i<numLegal; i++) newstates[i] = Copy(state);
	
	double pgs_values[numLegal]; //pgs_values[i] <-- legalMove[i]
	double pgs_state = PGS(*oldstate);	
		
	int max_p = -1;
	double max_v = -Infinity;	
	
	int observation;
	double reward;
	
	//cout << "Generating PGS values..." << endl;
	//cout << "Found " << numLegal << " legal actions." << endl;
	
	//#pragma omp parallel{
	
	//#pragma omp for
	for(unsigned int i=0; i<numLegal; i++){
		newstate = Copy(state);
		/* Used before to skip evaluating checks:
		if(acts[i] > E_SAMPLE){
			legal.push_back(acts[i]);
		}
		else*/		
		StepNormal(*newstate, acts[i], observation, reward); //Simulate transition with action a
		
		// Using regular PGS (slow)
		//pgs_values[i] = PGS(*newstate);
		
		// Adding only PGS differences (fast)
		pgs_values[i] = PGS_RO(*oldstate, *newstate, acts[i], pgs_state); //add differences

		FreeState(newstate);
		
			/*if(pgs_values[i] > max_v){
				max_v = pgs_values[i];
				max_p = i;
			}*/
//		FreeState(newstates[i]);
//			newstate = Copy(state);
		}
//	}
	
	FreeState(oldstate);
	max_p = std::distance(pgs_values, max_element(pgs_values, pgs_values+numLegal));
	max_v = pgs_values[max_p];
	assert(max_p > -1);
	
	legal.push_back(acts[max_p]); //Add best action to return vector
	// Add other maxima
	for(int i=0; i<numLegal; i++){
		if(i != max_p && pgs_values[i] == max_v)
		//if(pgs_values[i] >= 0.5)
			legal.push_back(acts[i]);
	}

	//cout << "found " << legal.size() << " rollout actions " << endl;
}

void ROCKSAMPLE::GeneratePGS_fake(const STATE& state, const HISTORY& history,
    vector<int>& legal, const STATUS& status) const
{
	const ROCKSAMPLE_STATE& rockstate = safe_cast<const ROCKSAMPLE_STATE&>(state);
	int rock = Grid(rockstate.AgentPos);
	
	if(rock >= 0){	
		if(!rockstate.Rocks[rock].Collected && rockstate.Rocks[rock].Count && rockstate.Rocks[rock].Valuable) legal.push_back(E_SAMPLE);
	}
	
	double p = 0.0;
	double binaryEntropy = 0.0;
	
	int observation;
	double distance;
	double efficiency;
	double likeV = 0.0;
	double likeW = 0.0;
	
	for(int rock=0; rock<NumRocks; rock++){
		if(rockstate.Rocks[rock].Measured){
			likeV = rockstate.Rocks[rock].LikelihoodValuable;
			likeW = rockstate.Rocks[rock].LikelihoodWorthless;
			
			observation = GetObservation(rockstate, rock);
			distance = COORD::EuclideanDistance(rockstate.AgentPos, RockPos[rock]);
			efficiency = (1 + pow(2, -distance / HalfEfficiencyDistance)) * 0.5;
			
			if(observation == E_GOOD){
				likeV *= efficiency;
				likeW *= 1.0 - efficiency;
			}else{         
				likeW *= efficiency;
				likeV *= 1.0 - efficiency;
			}
			
			p = (0.5 * rockstate.Rocks[rock].LikelihoodValuable) / (0.5*rockstate.Rocks[rock].LikelihoodValuable + 0.5*rockstate.Rocks[rock].LikelihoodWorthless);
					
			binaryEntropy = -1*p*log2(p) - (1-p)*log2(1-p);	
			if(binaryEntropy <= 0.5) legal.push_back(E_SAMPLE + rock + 1);
		}
		else
			legal.push_back(E_SAMPLE + rock + 1);		
	}
	
	//If no *preferred* actions were found
	if(legal.size() == 0){
		GenerateLegal(state, history, legal, status);
	}
}

void ROCKSAMPLE::GenerateLegal(const STATE& state, const HISTORY& history,
    vector<int>& legal, const STATUS& status) const
{

    const ROCKSAMPLE_STATE& rockstate =
        safe_cast<const ROCKSAMPLE_STATE&>(state);

    if (rockstate.AgentPos.Y + 1 < Size)
        legal.push_back(COORD::E_NORTH);

    legal.push_back(COORD::E_EAST);

    if (rockstate.AgentPos.Y - 1 >= 0)
        legal.push_back(COORD::E_SOUTH);

    if (rockstate.AgentPos.X - 1 >= 0)
        legal.push_back(COORD::E_WEST);

    int rock = Grid(rockstate.AgentPos);
    if (rock >= 0 && !rockstate.Rocks[rock].Collected)
        legal.push_back(E_SAMPLE);

    for (rock = 0; rock < NumRocks; ++rock)
        if (!rockstate.Rocks[rock].Collected)
            legal.push_back(rock + 1 + E_SAMPLE);
}

void ROCKSAMPLE::GeneratePreferred(const STATE& state, const HISTORY& history,
    vector<int>& actions, const STATUS& status) const
{	
	if(Knowledge.RolloutLevel >= KNOWLEDGE::PGS)
	{
		//Added alternative rollout policy for PGS
		GeneratePGS(state, history, actions, status);
		//GeneratePGS_fake(state, history, actions, status);
		//Legal moves may also be used.
		//GenerateLegal(state, history, actions, status);
	}
	else
	{
		static const bool UseBlindPolicy = false;

		if (UseBlindPolicy)
		{
			actions.push_back(COORD::E_EAST);
			return;
		}

		const ROCKSAMPLE_STATE& rockstate =
				  safe_cast<const ROCKSAMPLE_STATE&>(state);

		// Sample rocks with more +ve than -ve observations
		int rock = Grid(rockstate.AgentPos);
		if (rock >= 0 && !rockstate.Rocks[rock].Collected)
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
				actions.push_back(E_SAMPLE);
				return;
			}

		}

		// processes the rocks
		bool all_bad = true;
		bool north_interesting = false;
		bool south_interesting = false;
		bool west_interesting  = false;
		bool east_interesting  = false;

		for (int rock = 0; rock < NumRocks; ++rock)
		{
			const ROCKSAMPLE_STATE::ENTRY& entry = rockstate.Rocks[rock];
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

					if (RockPos[rock].Y > rockstate.AgentPos.Y)
						north_interesting = true;
					if (RockPos[rock].Y < rockstate.AgentPos.Y)
						south_interesting = true;
					if (RockPos[rock].X < rockstate.AgentPos.X)
						west_interesting = true;
					if (RockPos[rock].X > rockstate.AgentPos.X)
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
		if (rockstate.AgentPos.Y + 1 < Size && north_interesting)
				actions.push_back(COORD::E_NORTH);

		if (east_interesting)
			actions.push_back(COORD::E_EAST);

		if (rockstate.AgentPos.Y - 1 >= 0 && south_interesting)
			actions.push_back(COORD::E_SOUTH);

		if (rockstate.AgentPos.X - 1 >= 0 && west_interesting)
			actions.push_back(COORD::E_WEST);


		for (rock = 0; rock < NumRocks; ++rock)
		{
			if (!rockstate.Rocks[rock].Collected    &&
				rockstate.Rocks[rock].ProbValuable != 0.0 &&
				rockstate.Rocks[rock].ProbValuable != 1.0 &&
				rockstate.Rocks[rock].Measured < 5  &&
				std::abs(rockstate.Rocks[rock].Count) < 2)
			{
				actions.push_back(rock + 1 + E_SAMPLE);
			}
		}
	}
}

int ROCKSAMPLE::GetObservation(const ROCKSAMPLE_STATE& rockstate, int rock) const
{
    double distance = COORD::EuclideanDistance(rockstate.AgentPos, RockPos[rock]);
    double efficiency = (1 + pow(2, -distance / HalfEfficiencyDistance)) * 0.5;

    if (Bernoulli(efficiency))
        return rockstate.Rocks[rock].Valuable ? E_GOOD : E_BAD;
    else
        return rockstate.Rocks[rock].Valuable ? E_BAD : E_GOOD;
}

int ROCKSAMPLE::SelectTarget(const ROCKSAMPLE_STATE& rockstate) const
{
    int bestDist = Size * 2;
    int bestRock = -1;
    for (int rock = 0; rock < NumRocks; ++rock)
    {
        if (!rockstate.Rocks[rock].Collected
            && rockstate.Rocks[rock].Count >= UncertaintyCount)
        {
            int dist = COORD::ManhattanDistance(rockstate.AgentPos, RockPos[rock]);
            if (dist < bestDist)
                bestDist = dist;
        }
    }
    return bestRock;
}

void ROCKSAMPLE::DisplayBeliefs(const BELIEF_STATE& beliefState,
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

void ROCKSAMPLE::DisplayState(const STATE& state, std::ostream& ostr) const
{
    const ROCKSAMPLE_STATE& rockstate = safe_cast<const ROCKSAMPLE_STATE&>(state);
    ostr << endl;
    for (int x = 0; x < Size + 2; x++)
        ostr << "# ";
    ostr << endl;
    for (int y = Size - 1; y >= 0; y--)
    {
        ostr << "# ";
        for (int x = 0; x < Size; x++)
        {
            COORD pos(x, y);
            int rock = Grid(pos);
            const ROCKSAMPLE_STATE::ENTRY& entry = rockstate.Rocks[rock];
            if (rockstate.AgentPos == COORD(x, y))
                ostr << "* ";
            else if (rock >= 0 && !entry.Collected)
                ostr << rock << (entry.Valuable ? "$" : "X");
            else
                ostr << ". ";
        }
        ostr << "#" << endl;
    }
    for (int x = 0; x < Size + 2; x++)
        ostr << "# ";
    ostr << endl;
}

void ROCKSAMPLE::DisplayObservation(const STATE& state, int observation, std::ostream& ostr) const
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
    }
}

void ROCKSAMPLE::DisplayAction(int action, std::ostream& ostr) const
{
    if (action < E_SAMPLE)
        ostr << COORD::CompassString[action] << endl;
    if (action == E_SAMPLE)
        ostr << "Sample" << endl;
    if (action > E_SAMPLE)
        ostr << "Check " << action - E_SAMPLE << endl;
}
