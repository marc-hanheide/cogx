#ifndef DIALOGUE_ICE
#define DIALOGUE_ICE

// =================================================================
// Copyright (C) 2010 DFKI GmbH Talking Robots 
// Pierre Lison (pierre.lison@dfki.de)
// Geert-Jan M. Kruijff (gj@dfki.de)
// Hendrik Zender (hendrik.zender@dfki.de) 
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

#include <lf.ice>
#include <beliefs.ice>
#include <ref.ice>
#include <time.ice>
#include <abducer.ice>

// ===================================================================
// MODULE: de.dfki.lt.tr.dialogue.slice
// 
// Defines the data structures appearing on a working memory for a  
// subarchitecture processing situated dialogue. These data structures
// are organized into further modules: 
// 
// 
// Dependencies: 
// - de.dfki.lt.tr.dialogue.slice.lf, defined in [ ./lf.ice ] 
// 
// This Slice module is used with the dialogue API v6.0
// 
// @author Geert-Jan M. Kruijff	<gj@dfki.de>
// @author Pierre Lison	<pierre.lison@dfki.de>
// @author Hendrik Zender <hendrik.zender@dfki.de>
//
// ===================================================================

module de {
module dfki {
module lt { 
module tr { 
module dialogue { 
module slice { 

// ===================================================================
// TOP
// ===================================================================

	// BaseData
	// The class BaseData provides fields shared by all dialogue objects
	
	class BaseData { 
		string id;
	};  

	// Sequence of string Ids
	sequence<string> stringIds;

	["java:type:java.util.ArrayList<String>"] sequence<string> stringSeq;

	class StandbyMode {
	};

// ===================================================================
// MODULE ASR
// ===================================================================

module asr { 

	
	// ----------------------------------------------------------------------
	//  RecogResult
	//	The class RecogResult represents the result of a transaction with the Nuance
	//	Speech Recognition Server.  
	//
	//	@param boolean	isRecognized		true if something has been recognized by the ASR engine, false otherwise
	//	@param boolean	isConnectionClosed	true if the connection has been shut down by the caller, false otherwise
	//	@param string	recString		the string that has been recognized if isRecognized is true, or the empty string otherwise
	//	@param long	confidence		the confidence value of the result if there is one, or 0.0 otherwise
	//	@param long	probability		the probability value of the result if there is one, or 0.0 otherwise
	//	@param string	ipAddress		the IP address of the caller
	// ----------------------------------------------------------------------	

	class RecogResult extends BaseData {
		bool isRecognized ;
		bool isConnectionClosed ;
		string recString ;
		long confidence ;
		long probability ;
		string ipAddress ;
	  };	

	// ----------------------------------------------------------------------
	// PhonString
	// The class PhonString provides information about a phonological string, 
	// as recognized by a speech recognition engine.
	//
	// @param string	wordSequence	The recognized word sequence
	// @param long		length		The length of the word sequence (number of words)
	// @param float		confidenceValue	The confidence value
	// @param float     NLconfidenceValue   Confidence value used in feature selection	
	// @long  param		rank		The rank of a string (in an n-best list)
	// ----------------------------------------------------------------------

    class PhonString extends BaseData {
		string	wordSequence;
		long	length;
		float	confidenceValue;
		float   NLconfidenceValue;
		long	rank;
		bool	maybeOOV;
		time::Interval ival;
    };

	// ----------------------------------------------------------------------
	// Vector of PhonString objects
	// ----------------------------------------------------------------------

	sequence<PhonString> PhonStrings;

	// ----------------------------------------------------------------------
	// Noise
	// ----------------------------------------------------------------------

	class Noise {
		time::Interval ival;
	};

	class InitialPhonString {
		PhonString ps;
	};

	class InitialNoise {
		Noise n;
	};

}; 

// ===================================================================
// MODULE PARSE
// ===================================================================

module parse { 

	class NonStandardRule {
		string rulename;
		long numberOfApplications;
	};
	
	sequence<NonStandardRule> NonStandardRulesSeq;
	
	
	class NonStandardRulesAppliedForLF {
		string logicalFormId;
		NonStandardRulesSeq nonStandardRules;
	};
	
	sequence<NonStandardRulesAppliedForLF> NonStandardRulesAppliedForLFs; 

	class PhonStringLFPair {
		string logicalFormId;
		de::dfki::lt::tr::dialogue::slice::asr::PhonString phonStr;
	};
	
	sequence<PhonStringLFPair> PhonStringLFPairsSeq;
		
	//---------------------------------------------------------------------------	
	// The struct PackedLFs provides a data structure for information about 
	//	the logical form(s) that represent interpretations for the (given) 
	//	PhonString.
	//
	//	@param UniqueId	  id			The unique identifier of the object
	//	@param PhonString phon			The object representing the string (utterance) 
	//	@param PackedLogicalForm packedLF	The object representing the interpretations
	//	@param long finalized			Indicator whether the interpretations are finalized (0=unfinished; 1=finished parsing, 2=finished final pruning) 
	//---------------------------------------------------------------------------	

	class PackedLFs { 
		string id;
		PhonStringLFPairsSeq phonStringLFPairs;
		de::dfki::lt::tr::dialogue::slice::asr::PhonStrings nonParsablePhonStrings;
		long stringPos;
		lf::PackedLogicalForm packedLF;
		long finalized; 
		string type;
		NonStandardRulesAppliedForLFs nonStandardRulesForLF;
		float phonStringConfidence;
		bool phonStringMaybeOOV;
		time::Interval phonStringIval;
		string phonStringWordList;
	}; 

}; 

// ===================================================================
// MODULE PARSESELECTION
// ===================================================================

module parseselection {

	class SelectedLogicalForm {
		lf::LogicalForm lform;
		time::Interval ival;
		string phonStringWordList;
	};

};

// ===================================================================
// MODULE REF
// ===================================================================

module ref { 

	class NominalReference {
		string nominal;
		de::dfki::lt::tr::beliefs::slice::logicalcontent::dFormula referent;
	};

}; 

// ===================================================================
// MODULE DISCOURSE
// ===================================================================

module discourse {

	class DialogueMove {
		string agent;
		lf::LogicalForm lform;
		ref::NominalReference topic;
	};

};

// ===================================================================
// MODULE PRODUCE
// ===================================================================

module produce { 

	//---------------------------------------------------------------------------
	// Utterance planning and realization
	//---------------------------------------------------------------------------

	class ContentPlanningGoal { 
		string cpgid;
		lf::LogicalForm lform;

		// FIXME: hack
		ref::NominalReference topic;
	}; 
	
	
	class ProductionLF { 
		string plfid;
		lf::LogicalForm lform;

		// FIXME:: hack
		ref::NominalReference topic;
	}; 
	
}; 

// ===================================================================
// MODULE SYNTH
// ===================================================================

module synthesize {

	//---------------------------------------------------------------------------
	// Speech output
	//---------------------------------------------------------------------------

	class SpokenOutputItem extends BaseData { 
		string phonString;
		string inputStreamId;

		// FIXME: hack
		ref::NominalReference topic;
	}; 
	
}; 


// ===================================================================
// END MODULE DEFINITIONS / SLICE FILE
// ===================================================================

}; 
}; 


module beliefs {
module slice {
module intentions {


class CommunicativeIntention {
	Intention intent;
};


};
};
};

}; 
}; 
}; 
}; 

#endif
