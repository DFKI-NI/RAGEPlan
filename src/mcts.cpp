#include "mcts.h"
#include <math.h>

#include <algorithm>
#include <iomanip>

using namespace std;
using namespace UTILS;

//-----------------------------------------------------------------------------
MCTS::PARAMS::PARAMS()
:   Verbose(0),
    MaxDepth(100),
    NumSimulations(1000),
    NumStartStates(1000),
    UseTransforms(true),
    NumTransforms(0),
    MaxAttempts(0),
    ExpandCount(1),
    ExplorationConstant(1),
    DisableTree(false)
{
}

MCTS::MCTS(const SIMULATOR& simulator, const PARAMS& params)
:   Simulator(simulator),
    Params(params),
    TreeDepth(0)
{
    VNODE::NumChildren = Simulator.GetNumActions();
    QNODE::NumChildren = Simulator.GetNumObservations();
	
	 STATE* state;
	 if(Params.startstate)
		state = Params.startstate;
	 else
		state = Simulator.CreateStartState();
	 
    Root = ExpandNode(state);
	
		if (Params.Verbose >= 1){
			cout << "Simulator start state:" << endl;
			Simulator.DisplayState(*state, cout);
		}

    for (int i = 0; i < Params.NumStartStates; i++)
        Root->Beliefs().AddSample(Simulator.CreateStartState());
		
	/*** Incremental refinement ***/
	if(Params.useFtable){
		Simulator.initializeFTable(ftable);		
		//cout << "F-Table from sim received" << endl;
		if (Params.Verbose >= 1){
			//cout << "F-Table received (features = " << ftable.getNumFeatures() << ", actions = " << ftable.getNumActions() << ", entries = " << ftable.getNumEntries() << ")." << endl;
            cout << "F-Table received (entries = " << ftable.getNumEntries() << ")." << endl;
			if(Params.Verbose >= 2) cout << ftable;
		}
	}
	
}

MCTS::~MCTS()
{
    VNODE::Free(Root, Simulator);
    VNODE::FreeAll();
}

bool MCTS::Update(int action, int observation, double reward)
{
    History.Add(action, observation);
    BELIEF_STATE beliefs;

    // Find matching vnode from the rest of the tree
    QNODE& qnode = Root->Child(action);
    VNODE* vnode = qnode.Child(observation);
    if (vnode)
    {
        if (Params.Verbose >= 1)
            cout << "Matched " << vnode->Beliefs().GetNumSamples() << " states" << endl;        
        beliefs.Copy(vnode->Beliefs(), Simulator);
    }
    else
    {
        if (Params.Verbose >= 1)
            cout << "No matching node found" << endl;
    }

    // Generate transformed states to avoid particle deprivation
    if (Params.UseTransforms)
        AddTransforms(Root, beliefs);
    
    // If we still have no particles, fail
    if (beliefs.Empty() && (!vnode || vnode->Beliefs().Empty()))
        return false;

    if (Params.Verbose >= 2)
        Simulator.DisplayBeliefs(beliefs, cout);

	 /* After simulation and execution F-table should be revised:
			1) All beliefs corrected accordingly
			2) All tables in beliefs updated
	 */

	if(Params.useFtable)
		beliefRevision(beliefs);
	 
    // Find a state to initialise prior (only requires fully observed state)
    const STATE* state = 0;
    if (vnode && !vnode->Beliefs().Empty())
        state = vnode->Beliefs().GetSample(0);
    else
        state = beliefs.GetSample(0);	 

    // Delete old tree and create new root
    VNODE::Free(Root, Simulator);
    VNODE* newRoot = ExpandNode(state);
    newRoot->Beliefs() = beliefs;
    Root = newRoot;
    return true;
}

int MCTS::SelectAction()
{
    if (Params.DisableTree)
        RolloutSearch();
    else
        UCTSearch();

    int action;

    //cout << "Belief state: " << endl;
    //Simulator.DisplayBeliefs(Root->Beliefs(), cout);

    if(Params.useFtable)
        action = RelevanceUCB(Root, false); //f-aware UCB with IRE
    else
        action = GreedyUCB(Root, false); //Regular UCB

    return action;
}

void MCTS::RolloutSearch()
{
	std::vector<double> totals(Simulator.GetNumActions(), 0.0);
	int historyDepth = History.Size();
	std::vector<int> legal;
	assert(BeliefState().GetNumSamples() > 0);
	Simulator.GenerateLegal(*BeliefState().GetSample(0), GetHistory(), legal, GetStatus());
	random_shuffle(legal.begin(), legal.end());

	REWARD delayedReward;

	for (int i = 0; i < Params.NumSimulations; i++)
	{
		int action = legal[i % legal.size()];
		STATE* state = Root->Beliefs().CreateSample(Simulator);
		Simulator.Validate(*state);

		int observation;
		double immediateReward, totalReward, totalFReward;
		bool terminal = Simulator.Step(*state, action, observation, immediateReward);

		VNODE*& vnode = Root->Child(action).Child(observation);
		if (!vnode && !terminal)
		{
			vnode = ExpandNode(state);
			AddSample(vnode, *state);
		}
		History.Add(action, observation);

		delayedReward = Rollout(*state);
		totalReward = immediateReward + Simulator.GetDiscount() * delayedReward.V;
		totalFReward = immediateReward + Simulator.GetFDiscount() * delayedReward.F;

		Root->Child(action).Value.Add(totalReward);

		//NOTE: F-table update
		if(Params.useFtable && !terminal)
			ftable.valueUpdate(action, totalFReward);

		Simulator.FreeState(state);
		History.Truncate(historyDepth);
	}
}

void MCTS::UCTSearch()
{
    ClearStatistics();
    int historyDepth = History.Size();

    for (int n = 0; n < Params.NumSimulations; n++)
    {
        STATE* state = Root->Beliefs().CreateSample(Simulator);
        Simulator.Validate(*state);
        Status.Phase = SIMULATOR::STATUS::TREE;
        if (Params.Verbose >= 2)
        {
            cout << "Starting simulation" << endl;
            Simulator.DisplayState(*state, cout);
        }

        TreeDepth = 0;
        PeakTreeDepth = 0;
        REWARD reward = SimulateV(*state, Root);
        double totalReward = reward.V;
        StatTotalReward.Add(totalReward);
        StatTreeDepth.Add(PeakTreeDepth);
		  
        if (Params.Verbose >= 2)
            cout << "Total reward = " << totalReward << endl;
        if (Params.Verbose >= 3)
            DisplayValue(4, cout);

        Simulator.FreeState(state);
        History.Truncate(historyDepth);
    }

    DisplayStatistics(cout);
}

MCTS::REWARD MCTS::SimulateV(STATE &state, VNODE *vnode)
{
    int action = GreedyUCB(vnode, true);

    REWARD reward;

    PeakTreeDepth = TreeDepth;
    if (TreeDepth >= Params.MaxDepth) // search horizon reached
        return reward;

    if (TreeDepth == 1)
        AddSample(vnode, state);

    QNODE& qnode = vnode->Child(action);
    reward = SimulateQ(state, qnode, action);

    vnode->Value.Add(reward.V);
    
    return reward;
}

MCTS::REWARD MCTS::SimulateQ(STATE &state, QNODE &qnode, int action)
{
    int observation;
    REWARD reward, delayedReward;
    double immediateReward = 0;

    bool terminal = Simulator.Step(state, action, observation, immediateReward);
    assert(observation >= 0 && observation < Simulator.GetNumObservations());
    History.Add(action, observation);

    if (Params.Verbose >= 3)
    {
        Simulator.DisplayAction(action, cout);
        Simulator.DisplayObservation(state, observation, cout);
        Simulator.DisplayReward(immediateReward, cout);
        Simulator.DisplayState(state, cout);
    }

    VNODE*& vnode = qnode.Child(observation);
    if (!vnode && !terminal && qnode.Value.GetCount() >= Params.ExpandCount)
        vnode = ExpandNode(&state);

    if (!terminal)
    {
        TreeDepth++;
        if (vnode)
            delayedReward = SimulateV(state, vnode);
        else
            delayedReward = Rollout(state);
        TreeDepth--;
    }

    reward.V = immediateReward + Simulator.GetDiscount() * delayedReward.V;
    reward.F = immediateReward + Simulator.GetFDiscount() * delayedReward.F;
    qnode.Value.Add(reward.V);
	 
	//Update (f,a) value in f-table using discounted return F
	if(Params.useFtable && !terminal)
		ftable.valueUpdate(action, reward.F);

    return reward;
}

/*** Activate/deactivate objects in all beliefs ***/
//TODO: Determine activation policy
void MCTS::beliefRevision(BELIEF_STATE& beliefs){	
	std::vector<double> fvalues;
	ftable.getAllFValues(fvalues);
	float FTABLE_INACTIVE = ftable.getACTIVATION_THRESHOLD();

	if(Params.Verbose >= 1) {
        for (int i = 0; i < fvalues.size() - 1; i++) {
            cout << fvalues[i] << ", ";
        //    cout << "Useful?" << hvalues[i] << endl;
        }
        if (fvalues.size()) {
            cout << fvalues[fvalues.size() - 1] << "\t";
        //    cout << "Useful?" << hvalues[hvalues.size()] << endl;
        }
        cout << endl;
    }

    bool allOff = true;
	for(int i=0; i < fvalues.size(); i++) {
	    allOff = allOff && (fvalues[i] < FTABLE_INACTIVE);
        if (fvalues[i] < FTABLE_INACTIVE){
            beliefs.activateFeature(i, false);
            ftable.toggleActionsForFeature(i, false);
            if(Params.Verbose >= 1) cout << "Feature " << i << " is now OFF" << endl;
        }
        else{
            beliefs.activateFeature(i, true);
            ftable.toggleActionsForFeature(i, true);
            if(Params.Verbose >= 1) cout << "Feature " << i << " is now ON" << endl;
        }
    }

    ///If all features are off, activate one random feature (to get address estimation errors)
    if(allOff){
        int f = Random(fvalues.size());
        beliefs.activateFeature(f, true);
        ftable.toggleActionsForFeature(f, true);
        if(Params.Verbose >= 1) cout << "Feature " << f << " is back ON" << endl;
    }

	ftable.transition(); //Reset count and set prior to fvalue
	fvalues.clear();
}

VNODE* MCTS::ExpandNode(const STATE* state)
{
    VNODE* vnode = VNODE::Create();
    vnode->Value.Set(0, 0);
    Simulator.Prior(state, History, vnode, Status);

    if (Params.Verbose >= 2)
    {
        cout << "Expanding node: ";
        History.Display(cout);
        cout << endl;
    }

    return vnode;
}

void MCTS::AddSample(VNODE* node, const STATE& state)
{
    STATE* sample = Simulator.Copy(state);
    node->Beliefs().AddSample(sample);
    if (Params.Verbose >= 2)
    {
        cout << "Adding sample:" << endl;
        Simulator.DisplayState(*sample, cout);
    }
}

/*
  Relevance UCB is F-aware: Use only actions that apply to active features or according to rollout policy
*/
int MCTS::RelevanceUCB(VNODE *vnode, bool ucb) const
{
    static vector<int> besta;
    besta.clear();
    double bestq = -Infinity;
    int N = vnode->Value.GetCount();
    double logN = log(N + 1);

    vector<int> actions;
    //Relevance option 1: sample state and obtain active actions in that state
        /*STATE* state = vnode->Beliefs().CreateSample(Simulator);
        Simulator.GenerateRelevant(*state, History, actions, Status);
        Simulator.FreeState(state);
         */

    //Relevance option 2: eliminate actions of inactive features (safer)
    vector<int> inactiveActions;
    ftable.inactiveActions(inactiveActions);

    vector<int> allActions;
    for(int i=0; i<Simulator.GetNumActions(); i++) {
        allActions.push_back(i);
    }

    std::set_difference(allActions.begin(), allActions.end(), inactiveActions.begin(), inactiveActions.end(),
            std::inserter(actions, actions.begin()));

    if(Params.Verbose >= 1){
        cout << "UCB with " << actions.size() << " actions." << endl;
        //cout << "Belief sample (from " << vnode->Beliefs().GetNumSamples() << "):";
        //Simulator.DisplayState(*state, cout);
    }

    int action = 0;
    for (int i = 0; i < actions.size(); i++) {
        action = actions[i];

        double q;
        int n;

        QNODE& qnode = vnode->Child(action);
        q = qnode.Value.GetValue();
        n = qnode.Value.GetCount();
        
        if (ucb)
            q += FastUCB(N, n, logN);

        if (q >= bestq)
        {
            if (q > bestq)
                besta.clear();
            bestq = q;
            besta.push_back(action);
        }
    }

    actions.clear();
    assert(!besta.empty());
    return besta[Random(besta.size())];
}

int MCTS::GreedyUCB(VNODE* vnode, bool ucb) const
{
    static vector<int> besta;
    besta.clear();
    double bestq = -Infinity;
    int N = vnode->Value.GetCount();
    double logN = log(N + 1);

    for (int action = 0; action < Simulator.GetNumActions(); action++)
    {

        double q;
        int n;

        QNODE& qnode = vnode->Child(action);
        q = qnode.Value.GetValue();
        n = qnode.Value.GetCount();

        if (ucb)
            q += FastUCB(N, n, logN);

        if (q >= bestq)
        {
            if (q > bestq)
                besta.clear();
            bestq = q;
            besta.push_back(action);
        }

        if (Params.Verbose >= 2 && !ucb){
            cout << "\t";
            Simulator.DisplayAction(action, cout);
            cout << "Value = " << q << endl;
        }

        //cout << "UCB: a = " << action << ", q = " << q << endl;
    }

    assert(!besta.empty());
    return besta[Random(besta.size())];
}

MCTS::REWARD MCTS::Rollout(STATE &state)
{
    Status.Phase = SIMULATOR::STATUS::ROLLOUT;
    if (Params.Verbose >= 3)
        cout << "Starting rollout" << endl;

    REWARD rewardSt;

    double totalReward = 0.0;
    double totalF = 0.0;
    double discount = 1.0;
    double f_discount = 1.0;
    bool terminal = false;
    int numSteps;
    for (numSteps = 0; numSteps + TreeDepth < Params.MaxDepth && !terminal; ++numSteps)
    {
        int observation;
        double reward;

        int action = Simulator.SelectRandom(state, History, Status);
        terminal = Simulator.Step(state, action, observation, reward);
        History.Add(action, observation);

        if (Params.Verbose >= 4)
        {
            Simulator.DisplayAction(action, cout);
            Simulator.DisplayObservation(state, observation, cout);
            Simulator.DisplayReward(reward, cout);
            Simulator.DisplayState(state, cout);
        }

        totalReward += reward * discount;
        discount *= Simulator.GetDiscount();
    }

    rewardSt.V = totalReward;

    StatRolloutDepth.Add(numSteps);
    if (Params.Verbose >= 3)
        cout << "Ending rollout after " << numSteps
            << " steps, with total reward " << totalReward << endl;
    return rewardSt;
}

void MCTS::AddTransforms(VNODE* root, BELIEF_STATE& beliefs)
{
    int attempts = 0, added = 0;

    // Local transformations of state that are consistent with history
    while (added < Params.NumTransforms && attempts < Params.MaxAttempts)
    {
        STATE* transform = CreateTransform();

        if (transform)
        {
            beliefs.AddSample(transform);
            added++;
        }
        attempts++;
    }

    if (Params.Verbose >= 1)
    {
        cout << "Created " << added << " local transformations out of "
            << attempts << " attempts" << endl;
    }    
}

STATE* MCTS::CreateTransform() const
{
    int stepObs;
    double stepReward;
    
    STATE* state = Root->Beliefs().CreateSample(Simulator);
    Simulator.Step(*state, History.Back().Action, stepObs, stepReward);
    if (Simulator.LocalMove(*state, History, stepObs, Status))
        return state;
    
    Simulator.FreeState(state);
    return 0;
}

double MCTS::UCB[UCB_N][UCB_n];
bool MCTS::InitialisedFastUCB = true;

void MCTS::InitFastUCB(double exploration)
{
    cout << "Initialising fast UCB table... ";
    for (int N = 0; N < UCB_N; ++N)
        for (int n = 0; n < UCB_n; ++n)
            if (n == 0)
                UCB[N][n] = Infinity;
            else
                UCB[N][n] = exploration * sqrt(log(N + 1) / n);
    cout << "done" << endl;
    InitialisedFastUCB = true;
}

inline double MCTS::FastUCB(int N, int n, double logN) const
{
    if (InitialisedFastUCB && N < UCB_N && n < UCB_n)
        return UCB[N][n];

    if (n == 0)
        return Infinity;
    else
        return Params.ExplorationConstant * sqrt(logN / n);
}

void MCTS::ClearStatistics()
{
    StatTreeDepth.Clear();
    StatRolloutDepth.Clear();
    StatTotalReward.Clear();
}

void MCTS::DisplayStatistics(ostream& ostr) const
{
    if (Params.Verbose >= 1)
    {
        StatTreeDepth.Print("Tree depth", ostr);
        StatRolloutDepth.Print("Rollout depth", ostr);
        StatTotalReward.Print("Total reward", ostr);
    }

    if (Params.Verbose >= 2)
    {
        ostr << "Policy after " << Params.NumSimulations << " simulations" << endl;
        DisplayPolicy(6, ostr);
        ostr << "Values after " << Params.NumSimulations << " simulations" << endl;
        DisplayValue(6, ostr);
    }
}

void MCTS::DisplayValue(int depth, ostream& ostr) const
{
    HISTORY history;
    ostr << "MCTS Values:" << endl;
    Root->DisplayValue(history, depth, ostr);
}

void MCTS::DisplayPolicy(int depth, ostream& ostr) const
{
    HISTORY history;
    ostr << "MCTS Policy:" << endl;
    Root->DisplayPolicy(history, depth, ostr);
}
