package de.dfki.lt.tr.cast.dialogue.planverb;

import java.io.File;
import java.io.IOException;

public class StandaloneMessagesVerbalizer {

	/**
	 * main method for running stand-alone realization of PEV-generated messages!
	 * String grammarFile = args[0];
	 * String ngramFile = args[1];
	 * String hostname = args[2];
	 * Integer port = Integer.parseInt(args[3]);
	 * String messagesFileName = args[4];
	 * 
	 * 2012-07-11_16:18/PlanVerbMessages.xml
	 * pev/PlanVerbMessages.xml
	 * 
	 * @param arg
	 */
	public static void main(String[] args) {
		PlanVerbalizer test;
		try {
			String grammarFile = args[0];
			String ngramFile = args[1];
			String hostname = args[2]; 
			Integer port = Integer.parseInt(args[3]);
			String messagesFileName = args[4];
			
			test = new PlanVerbalizer(grammarFile, ngramFile, hostname, port);

			test.debug_lf_out = true;
			System.out.println(test.verbalizeMessageFile(new File(messagesFileName)));
		} catch (NumberFormatException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		} catch (IOException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
		
	}
}
