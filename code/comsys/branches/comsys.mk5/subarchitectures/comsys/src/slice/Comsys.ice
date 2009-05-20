#ifndef HWD_ICE
#define HWD_ICE

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

