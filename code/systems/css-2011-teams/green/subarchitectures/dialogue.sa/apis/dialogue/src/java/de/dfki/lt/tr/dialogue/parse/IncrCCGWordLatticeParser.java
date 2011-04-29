//=================================================================
// Copyright (C) 2006-2010 DFKI GmbH Talking Robots
// Geert-Jan M. Kruijff (gj@dfki.de)
// Pierre Lison (plison@dfki.de)
//
// This library is free software; you can redistribute it and/or
// modify it under the terms of the GNU Lesser General Public License
// as published by the Free Software Foundation; either version 2.1 of
// the License, or (at your option) any later version.
//
// This library is distributed in the hope that it will be useful, but
// WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
// Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public
// License along with this program; if not, write to the Free Software
// Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA
// 02111-1307, USA.
//=================================================================

//=================================================================
// PACKAGE DEFINITION 
package de.dfki.lt.tr.dialogue.parse;

//=================================================================
// IMPORTS

// Java
import java.util.Enumeration;
import java.util.Hashtable;
import java.util.Vector;

// OpenNLP
import opennlp.ccg.parse.ParseException;
import opennlp.ccg.parse.ParseResults;

// Dialogue API slice
import de.dfki.lt.tr.dialogue.slice.asr.PhonString;
import de.dfki.lt.tr.dialogue.slice.lf.*;

// Dialogue API 
import de.dfki.lt.tr.dialogue.asr.WordRecognitionLattice;
import de.dfki.lt.tr.dialogue.parse.PackedLFParseResults;
import de.dfki.lt.tr.dialogue.parse.IncrCCGStringParser;
import de.dfki.lt.tr.dialogue.util.DialogueMissingValueException;
import de.dfki.lt.tr.dialogue.util.LFPacking;

/**
 * The class <tt>IncrCCGWordLatticeParser</tt> implements an incremental
 * CCG parser that processes an entire word lattice, rather than just 
 * individual strings. The word lattice is constructed from the output of
 * automatic speech recognition. The result of incremental parsing is a
 * packed logical form, combining all the different LFs for the individual
 * (parseable) passes through the word lattice. 
 * <p>
 * The class provides a basic <tt>parse</tt> method, which simply 
 * provides a non-interrupted "complete" parse of a word lattice. The 
 * incremental parsing method is called using <tt>incrParse</tt> on the word lattice.
 * 
 * @see IncrCCGWordLatticeParser#parse(WordRecognitionLattice)
 * @see IncrCCGWordLatticeParser#incrParse(WordRecognitionLattice)
 * @see de.dfki.lt.tr.dialogue.asr.WordRecognitionLattice
 * @author Pierre Lison (plison@dfki.de)
 * @author Geert-Jan M. Kruijff (gj@dfki.de)
 * @version 100608
 */

public class IncrCCGWordLatticeParser 
extends IncrCCGStringParser {

	public int maxUtteranceLength = 10;

	Hashtable<String,PackedLFParseResults> alreadyFinishedParses ;

	/**
	 * Complete parsing for word recognition lattices. The method parses 
	 * the word lattice incrementally, from the first till the last position, 
	 * non-interruptable. 
	 * 
	 * @param 	lattice		  The word lattice to be parsed
	 * @return 	ParseResults  The parsing results
	 * @throws  DialogueMissingValueException Thrown if the lattice is null
	 * @throws ParseException  Passed up from lower parsing methods
	 */
	public ParseResults parse (WordRecognitionLattice lattice) 
	throws DialogueMissingValueException, ParseException 
	{
		if (lattice == null) 
		{
			throw new DialogueMissingValueException("Cannot parse word lattice: "+
					"Provided lattice is null");
		}
		// initialize return result
		PackedLFParseResults results = new PackedLFParseResults();
		// Run incremental parse steps over the length of the sentence
		alreadyFinishedParses = new Hashtable<String,PackedLFParseResults>();
		for (int i = 0; i < lattice.getMaximumLength() ; i++) { 
			results = (PackedLFParseResults) this.incrParse(lattice);
		} // end for over the length of the sentence
		return (ParseResults)results;
	} // end parse


	
	/**
	 * Incremental parsing for word recognition lattices
	 * 
	 * @param 	lattice		 The word lattice to be parsed
	 * @return	ParseResults The parsing results 
	 * @throws 	DialogueMissingValueException 	Thrown if the lattice is null
	 * @throws	ParseException	Passed up from lower parsing methods
	 */
	public ParseResults incrParse(WordRecognitionLattice lattice) 
	throws DialogueMissingValueException, ParseException
	{
		if (lattice == null) 
		{
			throw new DialogueMissingValueException("Cannot parse word lattice: "+
					"Provided lattice is null");
		}
		PackedLFParseResults results = new PackedLFParseResults();
		results.finalized = this.NOTFINISHED;
		Vector<PhonString> recogResults;
		if (lattice.getMaximumLength() <= maxUtteranceLength) 
		{
			recogResults = lattice.getAllResults();
		}
		else {
			recogResults = lattice.getFirstResult();
		} 
		Vector<LogicalForm> totalLFs = new Vector<LogicalForm>();
		boolean areParsesFinished = true;
		int incrStr = 1;
		String uniformNumbering = "";
		Hashtable<String,Hashtable<String,Integer>> nonStandardRules = 
			new Hashtable<String,Hashtable<String,Integer>>();
		results.phon2LFsMapping = new Hashtable<PhonString,Vector<String>>();
		results.nonParsablePhonStrings = new Vector<PhonString>();
		for (Enumeration<PhonString> e = recogResults.elements() ; e.hasMoreElements();) {
			PhonString phon = e.nextElement();
			results.phon2LFsMapping.put(phon, new Vector<String>());
			PackedLFParseResults result;
			result = (PackedLFParseResults) incrParse(phon);
			if (result != null && result.plf != null) 
			{
				for (Enumeration<String> f = result.nonStandardRulesApplied.keys();
				f.hasMoreElements();) {
					String ID = f.nextElement();
					nonStandardRules.put((ID+ "-" + incrStr), 
							result.nonStandardRulesApplied.get(ID));
				} // end for
				LFPacking packingTool2 = new LFPacking();
				LogicalForm[] unpackedLFs = packingTool2.unpackPackedLogicalForm(result.plf);
				Vector<String> vec = results.phon2LFsMapping.get(phon);
				for (int i = 0; i < unpackedLFs.length; i++) 
				{		
					unpackedLFs[i].logicalFormId = 
						unpackedLFs[i].logicalFormId + "-" + incrStr;
					vec.add(unpackedLFs[i].logicalFormId);
					for (int j = 0; j < unpackedLFs[i].noms.length ; j++) {
						LFNominal nom = unpackedLFs[i].noms[j];
						String[] split = nom.nomVar.split("_");
						if (split.length == 2) {
							if (uniformNumbering.equals("")) {
								uniformNumbering = split[1];
							}
							else {
								nom.nomVar = split[0] + "_" + uniformNumbering;
							}
						}
						for (int k = 0; k < nom.rels.length; k++) {
							LFRelation rel = nom.rels[k];
							String[] split2 = rel.head.split("_");
							rel.head = split2[0] + "_" + uniformNumbering;
							String[] split3 = rel.dep.split("_");
							rel.dep = split3[0] + "_" + uniformNumbering;
						} // end for over relations 
					} // end for over nominals in unpacked LFs
				} // end for over unpacked LFs
				Vector<LogicalForm> lfs = LFArrayToVector(unpackedLFs);				
				totalLFs.addAll(lfs);
				if (result.finalized == this.FINAL_PARSE) {
					alreadyFinishedParses.put(phon.id, result);
				}
				else {
					areParsesFinished = false;
				}
				incrStr++;
			}
			// phonological string is not parsable
			else {
				results.nonParsablePhonStrings.add(phon);
			}
		} // end for over recognition results
		LogicalForm[] totalLFsArray = new LogicalForm[totalLFs.size()];
		totalLFsArray = totalLFs.toArray(totalLFsArray);
		// Pack the LFs for the entire lattice
		PackedLogicalForm plf = packingTool.packLogicalForms(totalLFsArray);
		results.plf = plf;
		results.nonStandardRulesApplied = nonStandardRules;
		results.plf.packedLFId = "plf"+ utteranceIncrement;
		results.finalized = this.FINAL_PARSE;
		utteranceIncrement++;
		results.stringPos = -1;
		// Total number of logical forms for lattice = totalLFsArray.length
		return results;
	} // end incrParse

	private Vector<LogicalForm> LFArrayToVector(LogicalForm[] lfs) {
		Vector<LogicalForm> result = new Vector<LogicalForm>();
		for (int i = 0; i < lfs.length ; i++) {
			result.add(lfs[i]);
		}		
		return result;
	}

} // end class
