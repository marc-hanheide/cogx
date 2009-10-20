#ifndef COMSYS_ICE
#define COMSYS_ICE
 
// ========================= ===== =====================================
// MODULE: Comsys
// 
// Defines the data structures appearing on the comsys working memory 
// 
// Authors:		Geert-Jan M. Kruijff	<gj@dfki.de>
//				Pierre Lison			<pierre.lison@dfki.de>
//				Hendrik Zender			<hendrik.zender@dfki.de>
//
// ===================================================================

module speechrec {
module autogen {

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

