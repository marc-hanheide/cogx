
package comsys.processing.parse;

import opennlp.ccg.parse.ParseResults;

import java.util.ArrayList;
import java.util.Hashtable;
import java.util.Vector;

import comsys.datastructs.comsysEssentials.PhonString;
import comsys.datastructs.lf.PackedLogicalForm;

public class PackedLFParseResults 
	implements ParseResults 
{
	public PackedLogicalForm plf = null;
	
	public int finalized = 0;
	public int stringPos = 0;
	
	public Hashtable<PhonString,Vector<String>> phon2LFsMapping;
	public Vector<PhonString> nonParsablePhonStrings;
	public Hashtable<String,Hashtable<String,Integer>> nonStandardRulesApplied;
	
	public Hashtable<String,SignInChart> lfIdToSignMapping;
	
	public ArrayList<opennlp.ccg.synsem.Sign> parses;
	
	public ArrayList<opennlp.ccg.synsem.Sign> removedSigns;
	
	public PackedLFParseResults () {
		lfIdToSignMapping = new Hashtable<String,SignInChart>();
	}
	
	public PackedLFParseResults (PackedLogicalForm p) { 
		plf = p; 
		lfIdToSignMapping = new Hashtable<String,SignInChart>();
		}

	public void setStringPosition (int s) { stringPos = s; }
	
	
	public final class SignInChart {
		
		public int x;
		public int y;
		public opennlp.ccg.synsem.Sign sign;
		
	}
	
	

} // end class
