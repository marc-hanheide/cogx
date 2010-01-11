package comsys.components.tts;

import java.io.BufferedReader;
import java.io.FileReader;
import java.io.IOException;

import java.util.*;

import de.dfki.lt.mary.client.MaryClient;

public class GenerateWavFiles {

	private static String l_tuneA="A"; //is that a red@L* box HH%
	private static String l_tuneB="B"; //is that a red box@L* HH%
	private static String l_tuneC="C"; //is that@L* a red box HH%
	private static String l_tuneD="D"; //is@L* that a red box HH%
	
	private static String l_space= new String(" ");
	private static String l_Ls = new String ("@L*");
	private static String l_HHp = new String ("HH%");
	
	/**
	 * @param args
	 */
	public static void main(String[] args) {
		// TODO Auto-generated method stub
		String l_rawmary = new String("./subarchitectures/comsys/src/java/comsys/components/tts/RAWMARYXMLhead.xml");
		String l_outputloc = new String("./maryWavs/");
		
		ProsodicTextToRawMARYXml l_prs2xml = new ProsodicTextToRawMARYXml(l_rawmary,l_outputloc,"misc");
		
		try{
		MaryClient l_mary = new MaryClient("localhost", 59125);
		TTSLocal l_ttslocal = new TTSLocal(l_mary,"RAWMARYXML", "us2", false, "WAVE");
		SynthesisRAWMaryXMLInput l_synthsis = new SynthesisRAWMaryXMLInput(l_ttslocal);
		//Read the xls file
		
		
			
		
		//we need utterance, intonation type A, B, C, D, and file-name to save.
		
		FileReader l_xlsreader = new FileReader("/Users/raveesh/Documents/MSThesis/MyNotes/expr_cvs/final_allLists-Table 1.csv");
		BufferedReader l_xlsbfr = new BufferedReader(l_xlsreader);	
		String l_str = null;
		Integer l_ctr= 0;
		while((l_str= l_xlsbfr.readLine())!= null && l_ctr<=48){
			++l_ctr;
			if(l_ctr==1) continue; //First line is the table header, skip it
				
				
				System.out.println(l_ctr + ":"+ l_str);
				
				String l_itemcode = new String("");
				String l_uttr = new String("");
				String l_tune = new String("");
				String l_filename = new String("");
				String l_prs = new String("");
				StringTokenizer l_stkn = new StringTokenizer(l_str, ",");
				
				    l_itemcode=l_stkn.nextToken(); //Skip the 1st column
					l_uttr = l_stkn.nextToken(); //2nd column is the utterace
					l_stkn.nextToken(); //Skip the 3rd column
					l_tune = l_stkn.nextToken(); //4th column is the IntonationIndicatior
					l_stkn.nextToken(); //Skip the 5rd column 	 
					l_stkn.nextToken(); //Skip the 6th column 
					l_stkn.nextToken(); //Skip the 7th column 
					l_stkn.nextToken(); //Skip the 8th column 
					l_stkn.nextToken(); //Skip the 9th column 
					l_stkn.nextToken(); //Skip the 10th column 
					l_filename= l_stkn.nextToken(); //11th column has the wav/pic file name. 
					l_stkn.nextToken(); //Skip the 12th column 
					l_stkn.nextToken(); //Skip the 13th column 
				
					//We got the important stuff we needed
					
					
					//For A tune
					//Now convert the utterance to prosodic-utterance with the help of Intonation contour type.
					l_prs = Utterance2Prosodic(l_uttr, l_tuneA );
										
					// Specify the RAWMaryXML filename we want to use here
					l_prs2xml.g_xmlfilename=l_itemcode.concat("_").concat(l_tuneA).concat(".xml");
					
					//converts prosodic-utterance into RawMaryXML with above filename
					String l_xmlfile = new String();
					l_xmlfile=l_prs2xml.ConvertToRawMarxXml(l_prs);
					System.out.println("XML file written: ["+l_xmlfile+"]");
					
					
					// use Mary to generate a audio file and save it to disk
					l_ttslocal.m_AudioFileName=l_outputloc.concat(l_prs2xml.g_xmlfilename);
					l_ttslocal.m_SaveAudio2Wav=true;
					l_synthsis.Save2Wave(l_outputloc.concat(l_prs2xml.g_xmlfilename));
					
					//For B tune
					//Now convert the utterance to prosodic-utterance with the help of Intonation contour type.
					l_prs = Utterance2Prosodic(l_uttr, l_tuneB );
						
					// Specify the RAWMaryXML filename we want to use here
					l_prs2xml.g_xmlfilename=l_itemcode.concat("_").concat(l_tuneB).concat(".xml");
					
					//converts prosodic-utterance into RawMaryXML
					l_xmlfile=l_prs2xml.ConvertToRawMarxXml(l_prs);
					System.out.println("XML file written: ["+l_xmlfile+"]");
					
					
					// use Mary to generate a audio file and save it to disk
					l_ttslocal.m_AudioFileName=l_outputloc.concat(l_prs2xml.g_xmlfilename);
					l_ttslocal.m_SaveAudio2Wav=true;
					l_synthsis.Save2Wave(l_outputloc.concat(l_prs2xml.g_xmlfilename));
					
					
					//For C tune
					//Now convert the utterance to prosodic-utterance with the help of Intonation contour type.
					l_prs = Utterance2Prosodic(l_uttr, l_tuneC );
										
					// Specify the RAWMaryXML filename we want to use here
					l_prs2xml.g_xmlfilename=l_itemcode.concat("_").concat(l_tuneC).concat(".xml");
					
					//converts prosodic-utterance into RawMaryXML
					l_xmlfile=l_prs2xml.ConvertToRawMarxXml(l_prs);
					System.out.println("XML file written: ["+l_xmlfile+"]");
				
					// use Mary to generate a audio file and save it to disk
					l_ttslocal.m_AudioFileName=l_outputloc.concat(l_prs2xml.g_xmlfilename);
					l_ttslocal.m_SaveAudio2Wav=true;
					l_synthsis.Save2Wave(l_outputloc.concat(l_prs2xml.g_xmlfilename));
					
					//For B tune
					//Now convert the utterance to prosodic-utterance with the help of Intonation contour type.
					l_prs = Utterance2Prosodic(l_uttr, l_tuneD );
						
					// Specify the RAWMaryXML filename we want to use here
					l_prs2xml.g_xmlfilename=l_itemcode.concat("_").concat(l_tuneD).concat(".xml");
					
					//converts prosodic-utterance into RawMaryXML
					l_xmlfile=l_prs2xml.ConvertToRawMarxXml(l_prs);
					System.out.println("XML file written: ["+l_xmlfile+"]");
					
					// use Mary to generate a audio file and save it to disk
					l_ttslocal.m_AudioFileName=l_outputloc.concat(l_prs2xml.g_xmlfilename);
					l_ttslocal.m_SaveAudio2Wav=true;
					l_synthsis.Save2Wave(l_outputloc.concat(l_prs2xml.g_xmlfilename));
					
									
					//if (l_ctr == 2)break;
			}
		}
		catch (Exception e) {
        	e.printStackTrace();
		}
	}
	
	private static String Utterance2Prosodic(String i_utter, String i_tune){
		
		StringTokenizer l_uttertkn = new StringTokenizer(i_utter," ");
		String l_tkn = new String("");
		
		//System.out.println("input:" + i_utter + " tune: " +i_tune + " char:"+ i_tune.charAt(0));
		
		switch (i_tune.charAt(0)) {
		
		case 'A' : //Is that a red@L* box HH%
			while(l_uttertkn.hasMoreTokens()){
				l_tkn= l_tkn.concat(l_uttertkn.nextToken().toString()); // Is
				l_tkn= l_tkn.concat(l_space);
				l_tkn= l_tkn.concat(l_uttertkn.nextToken().toString()); //that
				l_tkn= l_tkn.concat(l_space);
				l_tkn= l_tkn.concat(l_uttertkn.nextToken().toString()); //a
				l_tkn= l_tkn.concat(l_space);
				l_tkn= l_tkn.concat(l_uttertkn.nextToken().toString()); //red@L*
				l_tkn= l_tkn.concat(l_Ls);
				l_tkn= l_tkn.concat(l_space);
				l_tkn= l_tkn.concat(l_uttertkn.nextToken().toString()); //box
				l_tkn= l_tkn.concat(l_space);
				l_tkn= l_tkn.concat(l_HHp); //HH%
			}
			return l_tkn;
			
		
		case 'B' : //Is that a red box@L* HH%
			while(l_uttertkn.hasMoreTokens()){
				l_tkn= l_tkn.concat(l_uttertkn.nextToken().toString()); // Is
				l_tkn= l_tkn.concat(l_space);
				l_tkn= l_tkn.concat(l_uttertkn.nextToken().toString()); //that
				l_tkn= l_tkn.concat(l_space);
				l_tkn= l_tkn.concat(l_uttertkn.nextToken().toString()); //a
				l_tkn= l_tkn.concat(l_space);
				l_tkn= l_tkn.concat(l_uttertkn.nextToken().toString()); //red
				l_tkn= l_tkn.concat(l_space);
				l_tkn= l_tkn.concat(l_uttertkn.nextToken().toString()); //box@L*
				l_tkn= l_tkn.concat(l_Ls);
				l_tkn= l_tkn.concat(l_space);
				l_tkn= l_tkn.concat(l_HHp);
			}
			return l_tkn;
			
		
		case 'C' : //Is that@L* a red box HH%
			while(l_uttertkn.hasMoreTokens()){
			l_tkn= l_tkn.concat(l_uttertkn.nextToken().toString()); // Is
			l_tkn= l_tkn.concat(l_space);
			l_tkn= l_tkn.concat(l_uttertkn.nextToken().toString()); //that@L*
			l_tkn= l_tkn.concat(l_Ls);
			l_tkn= l_tkn.concat(l_space);
			l_tkn= l_tkn.concat(l_uttertkn.nextToken().toString()); //a
			l_tkn= l_tkn.concat(l_space);
			l_tkn= l_tkn.concat(l_uttertkn.nextToken().toString()); //red
			l_tkn= l_tkn.concat(l_space);
			l_tkn= l_tkn.concat(l_uttertkn.nextToken().toString()); //box
			l_tkn= l_tkn.concat(l_space);
			l_tkn= l_tkn.concat(l_HHp);
		}
		return l_tkn;
		
		
		
		case 'D' :while(l_uttertkn.hasMoreTokens()){
			l_tkn= l_tkn.concat(l_uttertkn.nextToken().toString()); // Is@L*
			l_tkn= l_tkn.concat(l_Ls);
			l_tkn= l_tkn.concat(l_space);
			l_tkn= l_tkn.concat(l_uttertkn.nextToken().toString()); //that
			l_tkn= l_tkn.concat(l_space);
			l_tkn= l_tkn.concat(l_uttertkn.nextToken().toString()); //a
			l_tkn= l_tkn.concat(l_space);
			l_tkn= l_tkn.concat(l_uttertkn.nextToken().toString()); //red
			l_tkn= l_tkn.concat(l_space);
			l_tkn= l_tkn.concat(l_uttertkn.nextToken().toString()); //box
			l_tkn= l_tkn.concat(l_space);
			l_tkn= l_tkn.concat(l_HHp);
		}
		return l_tkn;
		
		default: System.out.println("Invalid tune. Tune has to be A, B , C or D type");
		}
		
		
		return "";
	}

}
