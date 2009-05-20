
package comsys.processing.parse;

import lf.PackedLogicalForm;
import comsys.*;
import opennlp.ccg.parse.ParseResults;
import java.util.Hashtable;
import java.util.Vector;

public class PackedLFParseResults 
	implements ParseResults 
{
	public PackedLogicalForm plf = null;
	public int finalized = 0;
	public int stringPos = 0;
	public Hashtable<PhonString,Vector<String>> phon2LFsMapping;
	
	public Vector<PhonString> nonParsablePhonStrings;
	
	public Hashtable<String,Hashtable<String,Integer>> nonStandardRulesApplied;
	
	public PackedLFParseResults () { }
	
	public PackedLFParseResults (PackedLogicalForm p) { plf = p; }

	public void setStringPosition (int s) { stringPos = s; }
	
	

} // end class
