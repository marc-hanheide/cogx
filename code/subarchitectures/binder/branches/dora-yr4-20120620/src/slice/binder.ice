// =================================================================
// Copyright (C) 2010 DFKI GmbH Talking Robots 
// Geert-Jan M. Kruijff (gj@dfki.de), Pierre Lison (pierre.lison@dfki.de) 
//                                                                                                                          
// This library is free software; you can redistribute it and/or
// modify it under the terms of the GNU Lesser General Public License 
// as published by the Free Software Foundation; either version 2.1 of
// the License, or (at your option) any later version.
//                                                                                                                          
// This library is distributed in the hope that it will be useful, but
// WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
// Lesser General Public License for more details.
//                                                                                                                          
// You should have received a copy of the GNU Lesser General Public
// License along with this program; if not, write to the Free Software
// Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA
// 02111-1307, USA.
// =================================================================
  
 

#ifndef BINDER_ICE
#define BINDER_ICE 


module org {
module cognitivesystems {
module binder {

	const string POINTERLABEL = "about";

	const string thisAgent = "self";
	const string humanAgent = "human";
	
module mln {

	struct Instance {
		string name;
		string type;
	};
	
	sequence<string> PredStrSeq;
	sequence<string> IdSeq;
	sequence<float> WeightSeq;
	sequence<float> ProbSeq;
	sequence<Instance> InstanceSeq;
	
	class Evidence {
		string engId;
		
		string setToken;
		string resetToken;
		
		InstanceSeq newInstances;
		InstanceSeq removeInstances;
		PredStrSeq trueEvidence;
		PredStrSeq falseEvidence;
		PredStrSeq noEvidence;
		PredStrSeq extPriors;
		WeightSeq priorWts;
		PredStrSeq resetPriors;
		
		int burnInSteps;
		int initInfSteps;
		int prevInfSteps;
//		int infSteps;
	};
	
	class Query {
		string engId;
		PredStrSeq atoms;
	};
	
	class InferredResult {
		string engId;
		PredStrSeq atoms;
		ProbSeq probs;
		string token;
		int tokenSamples; // number of samples since the token was introduced
		int overallSamples; // number of samples since the last evidence change (including tokens)
	};
	
	class LearnWts {
		string engId;
		PredStrSeq trueEvidence;
		PredStrSeq falseEvidence;
		PredStrSeq noEvidence;
	};
};

};
}; 
}; 

#endif
