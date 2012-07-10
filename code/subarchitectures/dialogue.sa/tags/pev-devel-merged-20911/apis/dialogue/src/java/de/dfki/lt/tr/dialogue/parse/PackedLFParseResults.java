//=================================================================
//Copyright (C) 2007-2010 Geert-Jan M. Kruijff (gj@dfki.de)

//This library is free software; you can redistribute it and/or
//modify it under the terms of the GNU Lesser General Public License
//as published by the Free Software Foundation; either version 2.1 of
//the License, or (at your option) any later version.

//This library is distributed in the hope that it will be useful, but
//WITHOUT ANY WARRANTY; without even the implied warranty of
//MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
//Lesser General Public License for more details.

//You should have received a copy of the GNU Lesser General Public
//License along with this program; if not, write to the Free Software
//Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA
//02111-1307, USA.
//=================================================================

//=================================================================
//PACKAGE DEFINITION
package de.dfki.lt.tr.dialogue.parse;

//=================================================================
//IMPORTS

//OpenCCG
import opennlp.ccg.parse.ParseResults;

//Java
import java.util.ArrayList;
import java.util.Hashtable;
import java.util.Vector;

//Dialogue API
import de.dfki.lt.tr.dialogue.slice.asr.PhonString;
import de.dfki.lt.tr.dialogue.slice.lf.PackedLogicalForm;

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