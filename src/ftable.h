/* F-Table v1.0
 * Incremental Refinement (IRE) uses a Feature-Table to manage feature-action pairs and their values.  
 * Feature values under some threshold are deactivated and their actions made unavailable in rollouts and relevance-UCB.
 * 
 */


#ifndef FTABLE_H
#define FTABLE_H

#include "utils.h"
#include <iostream>
#include <assert.h>
#include <ostream>

#include <set>

using std::cout;
using std::endl;

/*******************************************/
class FVALUE
{
public:

    void Set(double count, double value)
    {
        Count = count;
        Total = value * count;
    }

    void Add(double totalReward)
    {
        Count++;
        Total += totalReward;
    }

    void Add(double totalReward, double weight)
    {
        Count += weight;
        Total += totalReward * weight;
    }
	 
	 /*
		Learning rate value add, must track a non-stationary value
	 */
	void AlphaAdd(double totalReward, double alpha = 0.1, double gamma = 0.3){
        Count = 1;
        Total = Total + alpha*(gamma*totalReward - Total);
	}

    double GetValue() const
    {
        return Count == 0 ? Total : Total / Count;
    }
	 
    double GetTotal() const
    {
        return Total;
    }

    unsigned long GetCount() const
    {
        return Count;
    }

private:

    unsigned long Count = 0;
    double Total = 0.0;
};
/*******************************************/

class FTABLE{

public:
	struct F_ENTRY{
		int action;
		int feature;
		FVALUE value;
		bool active = true;

		//Testing
		FVALUE prior;
	};

	void valueUpdate(int action, double value); //Update all entries in tables for action a with value v
	//void validateTable(); //(De)Activate features according to their f-values

	/* Action/feature info */
	void toggleActionsForFeature(int feature, bool status);
	bool isFeatureActive(int f);
	bool isActionActive(int a);
	void inactiveActions(std::vector<int>& actions) const;

	/* Numerical f-values */
	double getFeatureValue(int feature); //Average the value of all entries for this feature
	void getAllFValues(std::vector<double>& fvalues); //Return vector with all feature values

	/* Table maintenance */
	void addEntry(F_ENTRY& entry);
	void addEntry(int action, int feature);

	void setTable(std::vector<F_ENTRY>& newTable);
	F_ENTRY& getEntry(int position);
	int getNumEntries();
	void clear();
	
	void inactivityUpdate();
	void transition();
	void reset();

	double getACTIVATION_THRESHOLD() const;
	void setACTIVATION_THRESHOLD(double ACTIVATION_THRESHOLD);

	void setNumFeatures(int numF) { NumFeatures = numF; }
	void setNumActions(int numA) { NumActions = numA; }
	int getNumActions() const { return NumActions; }
	int getNumFeatures() const { return NumFeatures; }
    void setTransitionRate(int tR);
    double getTransitionRate() const;

//private:
	//Tables for relevant and non-relevant features
	std::vector<F_ENTRY> Table;
private:
	double inactivity = -1.0;
	double ACTIVATION_THRESHOLD;
	double TRANSITION_RATE = 1.0; //0 - 1, similar to learning rate
	int NumActions;
    int NumFeatures;

protected:
    int INIT_VALUE = 0;
    int INIT_COUNT = 0;
    int NOACTION = -8; //Publication default

};

inline std::ostream& operator<<(std::ostream& ostr, FTABLE::F_ENTRY& f_entry){
	ostr << "<f = " << f_entry.feature << ", a = " << f_entry.action << ", v = [total = " << f_entry.value.GetValue() << ", count = " << f_entry.value.GetCount() << "]>" << endl;
	return ostr;
}

inline std::ostream& operator<<(std::ostream& ostr, FTABLE& ftable){
	int size = ftable.getNumEntries();
	ostr << "*** F-Table ***" << endl;

	for(int i=0; i < size; i++){
		ostr << "[" << i << "]" << ftable.getEntry(i);
	}
	ostr << "*** End ***" << endl;
    return ostr;
}

#endif
