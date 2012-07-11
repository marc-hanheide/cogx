package de.dfki.lt.tr.cast.dialogue.planverb;

import java.io.File;
import java.io.IOException;

import de.dfki.lt.tr.planverb.planning.pddl.PDDLHistory;

public class StandaloneHistoryVerbalizer {
	
	/**
	 * main method for running stand-alone PEV
	 * subarchitectures/dialogue.sa/resources/dora-interactive_annotated.txt subarchitectures/dialogue.sa/resources/domain2test.pddl subarchitectures/dialogue.sa/resources/grammars/openccg/moloko.v6/grammar.xml subarchitectures/dialogue.sa/resources/grammars/openccg/moloko.v6/ngram-corpus.txt localhost 4321 subarchitectures/dialogue.sa/resources/pev-test-data/2012-07-02_16:02/GBeliefHistory.xml subarchitectures/dialogue.sa/resources/pev-test-data/2012-07-02_16:02/history-1.pddl 1 
	 * other runs: 
	 * 2012-07-02_16:02 
	 * 2012-07-03_15:14 
	 * 2012-07-09_13:35 
	 * 2012-07-09_14:24 
	 * 2012-07-09_16:20 (has histories 1 and 3)
	 * 
	 * @param args, see above
	 */
	public static void main(String[] args) {
		File f = new File(args[7]);
		PlanVerbalizer test;
		try {
			test = new PlanVerbalizer(args[0], args[1], args[2], args[3], args[4], Integer.parseInt(args[5]), args[6]);
			test.debug_lf_out = true;
			System.out.println(test.verbalizeHistory(new PDDLHistory(f, Integer.parseInt(args[8]))));
		} catch (NumberFormatException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		} catch (IOException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
		
	}

}
