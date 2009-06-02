package comsys.processing.parse;

import opennlp.ccg.parse.ParseResults;
import opennlp.ccg.synsem.SignHash;

public class SignHashParseResults implements ParseResults{

	public SignHash hash = null; 

	public SignHashParseResults (SignHash h) { 
		hash = h;
	} // end constructor

} // end SignHashParseResults
