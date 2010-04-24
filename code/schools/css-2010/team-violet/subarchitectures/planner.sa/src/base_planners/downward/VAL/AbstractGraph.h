#ifndef __ABSTRACTGRAPH
#define __ABSTRACTGRAPH

#include "HowAnalyser.h"

namespace VAL {


class AbstractProposition {
private:
	extended_pred_symbol * eps;

public:
	AbstractProposition(extended_pred_symbol * e) : eps(e) 
	{};

};

class AbstractAction {
private:
	HWWAction * hww;
public:
	AbstractAction(HWWAction * h) : hww(h) 
	{};
};


class AbstractGraph {
private:
	vector<AbstractProposition *> factSpike;
	vector<AbstractAction *> actionSpike;

	vector<int> factLayerSizes;
	vector<int> actionLayerSizes;

	int layers;

	vector<AbstractAction *> acts;
	
public:
	AbstractGraph() : factLayerSizes(1,0), layers(0) {};
	~AbstractGraph()
	{
		for(vector<AbstractProposition*>::iterator i = factSpike.begin();
			i != factSpike.end();++i)
		{
			delete (*i);
		};
		for(vector<AbstractAction*>::iterator i = actionSpike.begin();
			i != actionSpike.end();++i)
		{
			delete (*i);
		};
	};
	
	void addInitialFact(extended_pred_symbol * eps)
	{
		factSpike.push_back(new AbstractProposition(eps));
		++factLayerSizes[0];
	};

	void addAction(HWWAction * h)
	{
		acts.push_back(new AbstractAction(h));
		cout << "Added action\n";
	};

	void develop()
	{
		factLayerSizes.push_back(0);
		while(extend())
		{
			cout << "Extended a layer\n";
			factLayerSizes.push_back(0);
			addNewFacts();
		};

		cout << "built\n";
	};

private:
	void addNewFacts()
	{};
	

	bool extend()
	{
		bool change = false;
		actionLayerSizes.push_back(0);
		for(unsigned int i = 0;i < acts.size();++i)
		{
			bool b = newlyApplicable(acts[i]);
			cout << "Acts: " << acts[i] << " " << b << "\n";
			if(acts[i] && b)
			{
				actionSpike.push_back(acts[i]);
				acts[i] = 0;
				++actionLayerSizes[layers];
				change = true;
			};
		};
		
		return change;
	};


	bool newlyApplicable(AbstractAction * a)
	{
		static int i = 0;
		++i;
		return i < 3;
	};

};


};


#endif
