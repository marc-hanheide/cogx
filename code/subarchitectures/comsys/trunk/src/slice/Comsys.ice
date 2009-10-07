#ifndef COMSYS_ICE
#define COMSYS_ICE

// ========================= ==========================================
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
#include <CDL.ice>
#include <BeliefModels.ice>
#include <Abducer.ice>
  
module comsys {
module datastructs {
module comsysEssentials {

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



 


	class SelectedLogicalForm extends lf::LogicalForm{

	PhonString phon;	
	double score;	
	double scoreOfSecondBest;
	bool exactMath;
	float partialMatch;
	};





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
	// Abstract structs for Caching - i.e. multi-level representation
	//---------------------------------------------------------------------------

	sequence<long> LongIds; 

	//	An index association relates two sequence of indices
	class IndexAssociation { 
		LongIds id1;
		LongIds id2;
		string relType ;
	} ;

	sequence<IndexAssociation> IndexAssociations;

	//	A CacheMapping is a sequence of associations between indices.
	class CacheMapping { 
		IndexAssociations associations;
	} ; 


	//	A Cache is a pair of pointers to content representations (one of which is a set of graph structures
	//	from the packed logical form) , and a mapping between indices in these representations 
	//	to associate content. 

	sequence<string> Pointers;

	class Cache {
		string CacheId ;  			// the cache identifier 
	    string cacheType ;			// the type of the cache, as per the comsys ontology types 
		PackedLFs plf ;				//  the packed logical form associated to the cache 
		Pointers content1 ;	// a set of pointers (eg. discourse referents) 
		Pointers content2 ;	// a set of pointers (eg. graph structures within the packed logical form) 
		CacheMapping mapping ;		// mapping between the indices of the two representations 
	} ;

	sequence<Cache> CacheSeq;

	//---------------------------------------------------------------------------
	// Structs for the discourse model representation
	//---------------------------------------------------------------------------
	
	sequence<string> StringArgs;
	
	
	// SDRS formula representing a rhetorical relation 
	class SDRSRelation {
		string relType ;		// Relation type 
		StringArgs args ;		// Sequence of arguments - speech act discourse referents 
		bool issubord ;			// is subordinated 
	} ;
	

	class SDRSType {
		string type ;
		SDRSRelation relation ;
		PackedLFs plf ;
	} ;			

	// A SDRS formula 
	class SDRSFormula {
		string label ;				// Formula label 
		string tprec ;				// Temporal precedence 
		CacheSeq caches ;				// set of caches associated to the formula 
		SDRSType type ;				// type of formula - either a rhetorical relation or a logical form 
	} ;
	

	// <label,formula> pair 
	class LabelFormulaPair {
		string label ;				// the label 
		SDRSFormula formula ;		// the formula 
	} ;
	
	sequence<LabelFormulaPair> LabelFormulaPairs;
	
	
	// Mapping between a label and a formula 
	class LabelFormulaMapping {
		LabelFormulaPairs mapping ;
	} ;
		
		
	// A SDRS - Segmented Discourse Representation Structure 
	class SDRS {
		StringArgs A ;			// Set of speech act discourse referents 
		string LAST ; 					// label of the content of the last clause  added to the LF 
		LabelFormulaMapping F ;			// function assigning a SDRS-formula to each label in A 
	} ;
	
	

	enum MoveType {
		ASSERT,
		ACTIONDIRECTIVE,
		QUESTIONW,
		QUESTIONYN,
		OPENING,
		CLOSING,
		ACCEPT,
		REJECT
	};
	
	class DialogueMove {
		string dmId;
		MoveType mType;
		string SDRSFormulaId1;
		string SDRSFormulaId2;
	};	

	//---------------------------------------------------------------------------
	// Structs for the spatio-temporal representation
	//---------------------------------------------------------------------------
	

	class EventDiscRefRelation {
		string eventId;
		string discRefId;
		string mode;
	};
	
	sequence<EventDiscRefRelation> EDRrelations;
	
	class Event {
		string eventId;
		string type;
		EDRrelations participants;
	};
	
	class StateDiscRefRelation {
		string stateId;
		string discRefId;
		string mode;
	};
	
	sequence<StateDiscRefRelation> SDRrelations;
	
	class State {
		string stateId;
		string type;
		SDRrelations participants;
	};
	
	class AspectualRelation {
		string eventSourceId;
		string stateTargetId;
		string aspect;
	};
	
	
	class TemporalRelation {
		string eventSourceId;
		string stateTargetId;
		string tempRelation;
	};
	
	sequence<Event> eventsSeq;
	sequence<State> statesSeq;
	sequence<AspectualRelation> aspectualRelationsSeq;
	sequence<TemporalRelation> temporalRelationsSeq;		
	
	class Nucleus {
		string nucleusId;
		string plfId;
		Cache  discRefs;
		eventsSeq events;
		statesSeq states;
		aspectualRelationsSeq aspectualRelations;
		temporalRelationsSeq temporalRelations;		
	};
	
	class InterNucleusRelation {
		string sourceId;
		string targetId;
		string mode;
	};
	
	sequence<Nucleus> nucleusSeq;
	sequence<InterNucleusRelation> interNucleusRelationSeq;	
	
	
	class SpatioTemporalRepresentation {
		nucleusSeq nuclei;
		interNucleusRelationSeq interNucleusRelations;
		string currentNucleusId;
	};
	





	
	//---------------------------------------------------------------------------
	// Utterance planning and realization
	//---------------------------------------------------------------------------

	class ContentPlanningGoal { 
		string cpgid;
		lf::LogicalForm lform;
	}; // end ProductionLF
	
	
	class ProductionLF { 
		string plfid;
		lf::LogicalForm lform;
	}; // end ProductionLF	
	
	class GRETask {
		string intendedReferentProxyID;
		bool done;
		string resultLF;
	};
	
	
	//---------------------------------------------------------------------------
	// Speech output
	// @param 
	//---------------------------------------------------------------------------

	class SpokenOutputItem extends BaseData { 
		string phonString;
        string inputStreamId ;
	}; // end SpokenOutputItem 
	
	
	
	//---------------------------------------------------------------------------
	// Reference determination and resolution 
	//---------------------------------------------------------------------------
	
	//---------------------------------------------------------------------------
	// A RefReading maintains two lists of id's. The identifiers refer to
	// (sub)trees in a logical form with id lfId. These subtrees are identified as providing
	// a restrictive form of reference, or an attributive form of reference. 
	//
	// @author	Geert-Jan Kruijff
	// @started	090921
	// @version	090921
	//---------------------------------------------------------------------------
	
	class RefReading extends BaseData { 
		string lfId;
		stringIds restrictiveTrees;
		stringIds attributiveTrees;
	}; 
	
	sequence<RefReading> Readings;

	class RefReadings extends BaseData {
		lf::LogicalForm lform; 
		Readings refRdngs; 
	}; 
	
	//---------------------------------------------------------------------------
	// A Anchor is a reference to a structure to which a reference can be 
	// anchored (identified by entityID), and probability of that anchoring
	// (probExists). 
	//
	// @author	Geert-Jan Kruijff
	// @started	090923
	// @version	090923
	//---------------------------------------------------------------------------	
	 
	class Anchor { 
		string entityID;
		float  probExists;	
	}; 
	
	sequence<Anchor> Anchors;
	
	//---------------------------------------------------------------------------
	// A RefBinding provides, for a given nominal (nomVar) in a given logical 
	// form (lfId), a list of antecedents to which it may be anchored. 
	//
	// @author	Geert-Jan Kruijff
	// @started	090923
	// @version	090923
	//---------------------------------------------------------------------------		
	
	class RefBinding extends BaseData { 
		string nomVar;
		Anchors antecedents;
	}; 
	
	sequence<RefBinding> RefBindings;
	
	//---------------------------------------------------------------------------
	// A ReadingBindings object provides for a given reading of a logical form, 
	// all the possible anchorings for the (restrictive) references in that reading. 
	//
	// @author	Geert-Jan Kruijff
	// @started	090923
	// @version	090923
	//---------------------------------------------------------------------------	
	
	class ReadingBindings extends BaseData { 
		RefBindings bindings; 
	}; 
	
	sequence<ReadingBindings> AnchoredReadings; 
	
	//---------------------------------------------------------------------------
	// A BoundReadings object provides a given logical form, the anchorings for 
	// the different restrictive references, per possible readings of the lf. 
	//
	// @author	Geert-Jan Kruijff
	// @started	090923
	// @version	090923
	//---------------------------------------------------------------------------		
	
	class BoundReadings extends BaseData { 
		lf::LogicalForm lform; 	
		AnchoredReadings bindings; 
	}; 
	
	//---------------------------------------------------------------------------
	// Continual acting
	//---------------------------------------------------------------------------
	
	sequence<beliefmodels::adl::Belief> DeltaSet;

	class ProofBlock {
		//Abducer::MarkedQuerySeq proof;
		string proofId;
		DeltaSet assumptions;
		DeltaSet assertions;
	};

	// Proof block stack, the top is the first element.
	sequence<ProofBlock> ProofBlockSeq;

	// A ClarificationRequest formulates a request which is a certain formula, 
	// (about) requesting the clarification of another formula relative to 
	// that formula (clarificationNeed). 
	// The about formula is grounded in the models of a source modality 
	// (sourceModality), referring to a structure in those models by an 
	// identifier (sourceEntityID). 
	
	class ClarificationRequest extends beliefmodels::adl::EpistemicObject 
	{ 
		beliefmodels::adl::Formula about;
		string  sourceModality;
		string  sourceEntityID;
		beliefmodels::adl::Formula clarificationNeed;
	}; 


};
}; 
};
#endif



// EDIT LOG
// 090930   janicek  Added ProofBlock, DeltaSet
// 090921	GJ	Added ReferentialReadings, ReferentialReading classes
// 090519	GJ	Added SpokenOutputItem: CHANGED: input stream is not an "any" object, but identified by id
// 090519	GJ	Added RecogResult
// 090518	GJ	Added PhonString class
// 090518	GJ	Defined BaseData as a class providing fields shared by all Comsys objects
// 090518	GJ	Started SLICE file; simplified name from ComsysEssentials to just comsys

