#ifndef COMSYS_ICE
#define COMSYS_ICE

// ===================================================================
// MODULE: Comsys
// 
// Defines the data structures appearing on the comsys working memory 
// 
// Authors:		Geert-Jan M. Kruijff	<gj@dfki.de>
//				Pierre Lison			<pierre.lison@dfki.de>
//				Hendrik Zender			<hendrik.zender@dfki.de>
//
// ===================================================================

#include <LF.ice>

module comsys {


	// BaseData 
	// The class BaseData provides fields shared by all Comsys objects

	class BaseData { 
		string id;
	};  

	// ----------------------------------------------------------------------
	// PhonString
	// The class PhonString provides information about a phonological string, 
	// as recognized by the speech recognition engine.
	//
	// @param string	wordSequence		The (best) recogized word sequence
	// @param long		length				The length of the word sequence (number of words)
	// @param float		confidenceValue		The confidence value	
	// @param float		NLconfidenceValue
	// @long  param		rank			
	// ----------------------------------------------------------------------

    class PhonString extends BaseData {
		string	wordSequence;
		long	length;
		float	confidenceValue;
		float	NLconfidenceValue;
		long	rank;
    };
	
	sequence<PhonString> PhonStrings;

	//---------------------------------------------------------------------------
	// Utterance processing
	//---------------------------------------------------------------------------	
	
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
		PhonString phonStr;
	};
	
	sequence<PhonStringLFPair> PhonStringLFPairsSeq;
		
	//---------------------------------------------------------------------------	
	// The struct PackedLFs provides a data structure for information about 
	//	the logical form(s) that represent interpretations for the (given) 
	//	PhonString.
	//
	//	@param UniqueId	  id					The unique identifier of the object
	//	@param PhonString phon				The object representing the string (utterance) 
	//	@param PackedLogicalForm packedLF	The object representing the interpretations
	//	@param long finalized				Indicator whether the interpretations are finalized (0=unfinished; 1=finished parsing, 2=finished final pruning) 
	//---------------------------------------------------------------------------	

	
	class PackedLFs { 
		string id;
		PhonStringLFPairsSeq phonStringLFPairs;
		PhonStrings nonParsablePhonStrings;
		long stringPos;
		lf::PackedLogicalForm packedLF;
		long finalized; 
		string type;
		NonStandardRulesAppliedForLFs nonStandardRulesForLF;
	}; // end PackedLFs



	// ----------------------------------------------------------------------
	//	The struct InterpretationSupport provides info about a single relation (in a PLF), and provides
	//	a list of supported or unsupported interpretations. Support is indicated by a boolean flag: "true" means
	//	the interpretation is supported, false means it is not. 
	// ----------------------------------------------------------------------

	sequence<string> stringIds;
	
	class InterpretationSupport { 
		string plfId;
		string headNomVar;
		string depNomVar;
		string mode;
		bool isSupported;
		stringIds LFids;		
	}; 

	sequence<InterpretationSupport> InterpretationSupports;

	// ----------------------------------------------------------------------
	//	The struct ContextInfo provides info about supported and unsupported interpretations for a given 
	//	packed logical form (by id). 
	// ----------------------------------------------------------------------
	 
	class ContextInfo { 
		string plfId; 
		InterpretationSupports interpretations;
	}; // end ContextInfo



	
	// ----------------------------------------------------------------------
	//  RecogResult
	//	The class RecogResult represents the result of a transaction with the Nuance
	//	Speech Recognition Server.  
	//
	//	@param boolean	isRecognized		true if something has been recognized by the ASR engine, false otherwise
	//	@param boolean	isConnectionClosed	true if the connection has been shut down by the caller, false otherwise
	//	@param string	recString			the string that has been recognized if isRecognized is true, or the empty string otherwise
	//	@param long		confidence			the confidence value of the result if there is one, or 0.0 otherwise
	//	@param long		probability			the probability value of the result if there is one, or 0.0 otherwise
	//	@param string	ipAddress			the IP address of the caller
	// ----------------------------------------------------------------------	

	class RecogResult extends BaseData {
		bool isRecognized ;
		bool isConnectionClosed ;
		string recString ;
		long confidence ;
		long probability ;
		string ipAddress ;
	  };	

	
	//---------------------------------------------------------------------------
	// Utterance planning and realization
	//---------------------------------------------------------------------------

	class ContentPlanningGoal { 
		string cpgid;
		lf::LogicalForm lform;
	}; // end ProductionLF
	
	
	
	//---------------------------------------------------------------------------
	// Speech output
	// @param 
	//---------------------------------------------------------------------------

	class SpokenOutputItem extends BaseData { 
		string phonString;
        string inputStreamId ;
	}; // end SpokenOutputItem 
	
	
};

#endif



// EDIT LOG
// 090519	GJ	Added SpokenOutputItem: CHANGED: input stream is not an "any" object, but identified by id
// 090519	GJ	Added RecogResult
// 090518	GJ	Added PhonString class
// 090518	GJ	Defined BaseData as a class providing fields shared by all Comsys objects
// 090518	GJ	Started SLICE file; simplified name from ComsysEssentials to just comsys

