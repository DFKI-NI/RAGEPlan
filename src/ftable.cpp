#include "ftable.h"

//Update all entries in tables for action a with value v
void FTABLE::valueUpdate(int action, double value){
	int weight = 1;
	for(int i=0; i < Table.size(); i++){
		if(Table[i].action == action){
			Table[i].value.Add(value);
		}
	}
}

void FTABLE::inactivityUpdate(){
	for(int i=0; i < Table.size(); i++){	
		Table[i].value.Add(-10);
	}
}

void FTABLE::reset(){
	for(int i=0; i < Table.size(); i++){	
		Table[i].value.Set(0,0);
        Table[i].prior.Set(0,0);
	}
}

/* 
 * Update the table after executing 'real' action.
 * On each entry the count is reset to 1 and the value reset to the current value discounted by the transition rate (1 = 100% of prior value).
 * Useful in domains where feature values may change drastically.
 */
void FTABLE::transition(){
	double val = 0;
	for(int i=0; i < Table.size(); i++){
	    if(Table[i].value.GetCount()) {
            val = Table[i].value.GetValue();// * FTABLE::TRANSITION_RATE;
            Table[i].value.Set(1, val);
        }
	}
}

/*
 * Enable/disable all actions associated with a feature
 *
 */
void FTABLE::toggleActionsForFeature(int feature, bool status){
    for(int i=0; i < Table.size(); i++){
        if(Table[i].feature == feature) {
            Table[i].active = status;
        }
    }
}

/*
 * Return status of action 'a' if in Table, otherwise return true.
 * */
bool FTABLE::isActionActive(int a){
    for(int i=0; i < Table.size(); i++){
        if(Table[i].action == a)
            return Table[i].active;
    }
    return true;
}

void FTABLE::inactiveActions(std::vector<int>& actions) const{
    for(int i=0; i < Table.size(); i++){
        if(Table[i].active == false)
            actions.push_back(Table[i].action);
    }
}

//Q(f,.) = Feature value is the average of all entries that contain this feature
/*
 * Useful features: have one very useful action or several useful actions
 * Nonuseful features: have more useless than useful actions, no very useful actions.
 * */
double FTABLE::getFeatureValue(int feature){
	double max  = -Infinity;
    double total = 0.0;
    double val = 0.0;
    double exp = 2.0; //Exponent to poly side of value function
	double totalPos = 0.0;
	double totalNeg = 0.0;
	double weightPos = 1.0;
	double weightNeg = 1.0;
	int count = 0;

	/* Linear negative + poly positive
	 *
	 *  Negative values are simply added, unused actions are punished (NOACTION)
	 *  and positive values are amplified exponentially/polynomially
	 * */

    for(int i=0; i < Table.size(); i++) {
        if (Table[i].feature == feature) {
            if (Table[i].value.GetCount()) {
                val = Table[i].value.GetValue();
                if(val > 1) val = pow(val, exp);

                total += val;
            }
            else {
                total += FTABLE::NOACTION; //If action hasn't been executed, penalize
            }
            count++;
        }
    }

    if(count > 0)
        total /= count;

	return total;
}

void FTABLE::addEntry(int action, int feature){
	F_ENTRY entry;
	entry.action = action;
	entry.feature = feature;
	entry.value.Set(FTABLE::INIT_COUNT,FTABLE::INIT_VALUE);

    //Set prior
    entry.prior.Set(FTABLE::INIT_COUNT,FTABLE::INIT_VALUE);

	Table.push_back(entry);
	//cout << "Added: " << entry << endl;
}

void FTABLE::addEntry(F_ENTRY& entry){
	Table.push_back(entry);
}

void FTABLE::setTable(std::vector<F_ENTRY>& newTable){
	Table.clear();
	
	for(std::vector<F_ENTRY>::iterator it = newTable.begin(); it != newTable.end(); ++it){	
		Table.push_back(*it);
	}
	
}

FTABLE::F_ENTRY& FTABLE::getEntry(int position){
	assert(position >= 0 && position < Table.size());
	return Table[position];
}

int FTABLE::getNumEntries(){
	return Table.size();
}

void FTABLE::clear(){
	Table.clear();
}

/*
 *  getAllFeatureValues v2.0:
 *      Traverse the table only once, adding all weighted feature values and pushing them in fvalues.
 *      Equivalent but much more efficient than old version.
 */
void FTABLE::getAllFValues(std::vector<double>& fvalues){
    std::vector<FVALUE> f_vals(NumFeatures);
    double val = 0.0;
    double exp = 2.0; //Exponent to poly side of value function
    
    for(auto fv : f_vals) fv.Set(0,0);

	/* Linear negative + poly positive
	 *
	 *  Negative values are simply added, unused actions are punished (NOACTION)
	 *  and positive values are amplified exponentially/polynomially
	 * */

    int f = 0;
    for(auto E : Table){
        f = E.feature;
        val = 0.0;
        if(E.value.GetCount()) {
            val = E.value.GetValue(); //Negative values are preserved
            if(val > 0) val = pow(val, exp); //Positive values are amplified
        }
        else val = FTABLE::NOACTION; //If action hasn't been executed, penalize
        
        f_vals[f].Add(val); //Finally, add value to corresponding feature
    }
    
    for(auto fv : f_vals)
        fvalues.push_back(fv.GetValue());
}

double FTABLE::getACTIVATION_THRESHOLD() const {
    return ACTIVATION_THRESHOLD;
}

void FTABLE::setACTIVATION_THRESHOLD(double ACTIVATION_THRESHOLD) {
    FTABLE::ACTIVATION_THRESHOLD = ACTIVATION_THRESHOLD;
}
