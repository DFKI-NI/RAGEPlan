#include "beliefstate.h"
#include "simulator.h"
#include "utils.h"

using namespace UTILS;

BELIEF_STATE::BELIEF_STATE()
{
    Samples.clear();
}

void BELIEF_STATE::Free(const SIMULATOR& simulator)
{
    for (std::vector<STATE*>::iterator i_state = Samples.begin();
            i_state != Samples.end(); ++i_state)
    {
        simulator.FreeState(*i_state);
    }
    Samples.clear();
}

STATE* BELIEF_STATE::CreateSample(const SIMULATOR& simulator) const
{
    int index = Random(Samples.size());
    return simulator.Copy(*Samples[index]);
}

void BELIEF_STATE::AddSample(STATE* state)
{
    Samples.push_back(state);
}

/*
	TODO: Moving feature activation here would perform one less loop
*/
void BELIEF_STATE::Copy(const BELIEF_STATE& beliefs, const SIMULATOR& simulator)
{
    for (std::vector<STATE*>::const_iterator i_state = beliefs.Samples.begin();
        i_state != beliefs.Samples.end(); ++i_state)
    {
        //TODO: Copy state once features have been revised
		  //(**i_state).validateTable();
        AddSample(simulator.Copy(**i_state));
    }
}

void BELIEF_STATE::Move(BELIEF_STATE& beliefs)
{
    for (std::vector<STATE*>::const_iterator i_state = beliefs.Samples.begin();
        i_state != beliefs.Samples.end(); ++i_state)
    {
        AddSample(*i_state);
    }
    beliefs.Samples.clear();
}

void BELIEF_STATE::activateFeature(int feature, bool status){
    for (std::vector<STATE*>::const_iterator i_state = Samples.begin();
        i_state != Samples.end(); ++i_state){

        (*i_state)->activateFeature(feature, status);
    }
}
