package comsys.components.tts;

import java.io.BufferedReader;
import java.io.BufferedWriter;
import java.io.File;
import java.io.FileReader;
import java.io.FileWriter;
import java.io.IOException;

import java.util.*;

import de.dfki.lt.mary.client.MaryClient;

public class RavGenerateWavFiles {
	private static Boolean debug=true;
	private static String g_Lang="";
	private static String l_tuneA="A"; //is that a red@L* box HH%
	private static String l_tuneB="B"; //is that a red box@L* HH%
	private static String l_tuneC="C"; //is that@L* a red box HH% , this maps to fA
	private static String l_tuneD="D"; //is@L* that a red box HH% , this maps to fB
	
	private static String l_space= new String(" ");
	private static String l_Ls = new String ("@L*");
	private static String l_HHp = new String ("HH%");
	public static LinkedHashMap<String, String> l_audioRsrc_1 = new LinkedHashMap<String, String>();
	public static LinkedHashMap<String, String> l_audioRsrc_2 = new LinkedHashMap<String, String>();
	public static LinkedHashMap<String, String> l_audioRsrc_3 = new LinkedHashMap<String, String>();
	public static LinkedHashMap<String, String> l_audioRsrc_4 = new LinkedHashMap<String, String>();
	public static LinkedHashMap<String, String> l_audioRsrc_5 = new LinkedHashMap<String, String>();
	public static LinkedHashMap<String, String> l_audioRsrc_6 = new LinkedHashMap<String, String>();
	public static LinkedHashMap<String, String> l_audioRsrc_7 = new LinkedHashMap<String, String>();
	public static LinkedHashMap<String, String> l_audioRsrc_8 = new LinkedHashMap<String, String>();
	public static LinkedHashMap<String, String> l_audioRsrc_0 = new LinkedHashMap<String, String>();
	
	public static LinkedHashMap<String, String> l_imageRsrc_1 = new LinkedHashMap<String, String>();
	public static LinkedHashMap<String, String> l_imageRsrc_2 = new LinkedHashMap<String, String>();
	public static LinkedHashMap<String, String> l_imageRsrc_3 = new LinkedHashMap<String, String>();
	public static LinkedHashMap<String, String> l_imageRsrc_4 = new LinkedHashMap<String, String>();
	public static LinkedHashMap<String, String> l_imageRsrc_5 = new LinkedHashMap<String, String>();
	public static LinkedHashMap<String, String> l_imageRsrc_6 = new LinkedHashMap<String, String>();
	public static LinkedHashMap<String, String> l_imageRsrc_7 = new LinkedHashMap<String, String>();
	public static LinkedHashMap<String, String> l_imageRsrc_8 = new LinkedHashMap<String, String>();
	public static LinkedHashMap<String, String> l_imageRsrc_0 = new LinkedHashMap<String, String>();
	
	public static LinkedHashMap<String, String> l_audioHumRes = new LinkedHashMap<String, String>();
	
	public static LinkedHashMap<String, String> l_Eng2De = new LinkedHashMap<String, String>();
	
	public static Vector<LinkedHashMap> l_allImageRes = new Vector<LinkedHashMap>();
	public static Vector<LinkedHashMap> l_allAudioRes = new Vector<LinkedHashMap>();
	private static Random randomGenerator = new Random();
	private static String l_outputloc;
	private static String l_outputloc_mary;
	private static String l_outputloc_rsrc;
	/**
	 * @param args The main() takes "a csv" file as input and generates the following:
	 * 1. the mary wav files (and rawxml), "Is that a red box?"
	 * 2. a script to merge these generated file with a YES or NO response (mentioned in exp specification)"Is that a red box. No"
	 * 		This requires NO.wav and YES.wav in the directory where you will execute the script.
	 * 		File name: mrgRobotHumanRes.sh
	 * 		contains command like: sox 12_A.wav NO.wav 12_A_NO.wav
	 * 		where 12_A_NO.wav is the file for "Is that a red box. No"
	 * 3. WebExp resources file like 
	 * 		a) File Name: exp-list1-images.xml, 
	 * 			Contains like:  <resource id="img01">bla-cub_bla-cir_lef.jpg</resource>
	 * 					   <resource id="img02">bla-cub_bla-cub_rig.jpg</resource> 
	 * 		b) File Name: exp-list1-sounds.xml
	 * 			contains like:  <resource id="01snd">12_B_YES.wav </resource>
	 * 					   <resource id="02snd">06_B_NO.wav </resource>
	 * generation of 3 is optional as indicated by input WebExp
	 * however it mandates flags WavFiles, which is also 2 above 
	 * 
	 * @csv csv file : each stimuli xls sheets is manually stored as csv file. This file is a input to this program.
	 * @output output location to store generated files
	 * @WavFiles flag to indicate generation of mary wav files (and xmls only). Stored in output/maryGen. MaryServer must be running at port
	 * 56125, on localhost. We are using voice us2 as default (hardcoded here)
	 * @WebExp flag to indicate generation of WebExp resource lists. Stored in output/rswebExp
	 */
	public static void main(String[] args) {
		// TODO Auto-generated method stub
		//This script can also be used for German lang.
		String l_rawmary="";
		g_Lang=args[0];
		if(g_Lang.contains("de")) {
			l_rawmary = new String("./subarchitectures/comsys/src/java/comsys/components/tts/DE_RAWMARYXMLhead.xml");	
		}
		else l_rawmary = new String("./subarchitectures/comsys/src/java/comsys/components/tts/ENG_RAWMARYXMLhead.xml");
		
		//xml:lang="en">
		String l_translate = new String("/home/spaniard81/mydata/msclst/ms_thesis/WebExp/evalThesis/xlsheet/EngToGer.csv");
		
		//	Integer l_numOfRows=98;
		try{
			//This is dumb, but we keep a mapping of English vs German sentences.
		FileReader l_eng2deRead = new FileReader(l_translate);
		BufferedReader l_eng2deBfr = new BufferedReader(l_eng2deRead);
		String l_eng2de = null;
		//Read each row in this csv file, and process as per columan values.
		while((l_eng2de= l_eng2deBfr.readLine())!= null){
			StringTokenizer l_eng2de_stkn = new StringTokenizer(l_eng2de, ",");
			l_Eng2De.put(replaceQuotes(l_eng2de_stkn.nextToken()), replaceQuotes(l_eng2de_stkn.nextToken()));
		}
		
		//Here goes the remaining processing.
		FileReader l_CSVreader = new FileReader(args[1]);
		l_outputloc = new String(args[2]);
		l_outputloc_mary = new String(l_outputloc.concat("/"+g_Lang+"/").concat("maryGen/"));
		l_outputloc_rsrc = new String(l_outputloc.concat("/"+g_Lang+"/").concat("rswebExp/"));
		
		ProsodicTextToRawMARYXml l_prs2xml = new ProsodicTextToRawMARYXml(l_rawmary,l_outputloc_mary,"misc");
		
		
		MaryClient l_mary = new MaryClient("localhost", 59125);
		//MaryClient l_mary = MaryClient.getMaryClient();
		TTSLocal l_ttslocal = new TTSLocal(l_mary,"RAWMARYXML", g_Lang, false, "WAVE");
		SynthesisRAWMaryXMLInput l_synthsis = new SynthesisRAWMaryXMLInput(l_ttslocal);
		
		//we need utterance, intonation type A, B, fA, fB, and file-name to save.
		
		boolean b_sndFiles =false;
		boolean b_webExpLst=false;
		//Check if Mary Wav needs to be generated.
		if(args[3].equals("WavFiles")) {
			b_sndFiles=true;
		}
		//Check if WebExo resource files needs to be generated.
		if(args[4].equals("WebExp")) {
			if(!args[3].equals("WavFiles")) {
				System.err.println("3rd parameter should be WavFiles.");
				System.exit(0); //terminate normally
			}
			b_webExpLst=true;
		}
		
		//Read the specified csv file
		BufferedReader l_CSVbfr = new BufferedReader(l_CSVreader);	
		String l_str = null;
		Integer l_ctr= 0;
		//Read each row in this csv file, and process as per columan values.
		//while((l_str= l_CSVbfr.readLine())!= null && l_ctr<=l_numOfRows){
		while((l_str= l_CSVbfr.readLine())!= null){
			++l_ctr;
			if(l_ctr==1) continue; //First row is the header so skip it
				
			//declare valriable for the columns we are interested in.
			System.out.println(l_ctr + ":"+ l_str);
			String l_list= new String("");
			String l_itemcode = new String("");
			String l_itemNum = new String(""); //itemcode stripped of a,b,c,d...
			
			String l_uttr = new String(""); //Utterance for Mary
			String l_tune = new String(""); //Intonation for this Utterance, Mary will use it
			String l_response = new String(""); //What is the human response, Yes or No
												//We will append them to Mary output with a script
			//l_itemNum+l_tune+l_response is used in  WebExp resource list for sound files.
			
			String l_scenename = new String(""); //Picture associated with this Utterance, Used for WebExp resource list
			String l_prs = new String("");
			
			//Since the file is a Comma Seperated File, tokenize them with a comma.
			StringTokenizer l_stkn = new StringTokenizer(l_str, ",");
			
			//Rest of the processing very much depends on the CSV columsn. Any change in CSV columns will require changes here
			//Each string has been stored a "I am a string" in the csv, strip quotes.
			l_stkn.nextToken(); //the 1st column is skipped
			l_itemcode= replaceQuotes(l_stkn.nextToken()); // 2nd column is itemcode
			
			//CSV file has been modified for this column, the itemcode can be 3 or 4 literals, e.g. 12c and f12c 
			/* The reason we strip itemcode of these literal is that in principle for an utterance "Is that a red box",
			 *  we have only 4 main intonation tunes. Generating a sound file with name having itemcode as such will result into
			 *  lots of duplicate file on the WebExp server, we want to reduce that load. So, generate a sound file 
			 *  for this utterance for these 4 basic patterns, with name: itemNum+Tune.
			 *  Generate another script to concatenate them with Yes or No responses as indicated 
			 *  in the csv file, and store this in the resource list with names: itemNum+tune+response.
			*/
			
			if(l_itemcode.length()==3) {
				l_itemNum=l_itemcode.substring(0, 2); //strip the suffic literals a,b,c...
				//System.out.println("item :"+ l_itemNum +":");
					
			}else if(l_itemcode.length()==4){
				l_itemNum=l_itemcode.substring(1, 3); //strip the suffic literals a,b,c...and prefix literal f:filler
				//System.out.println("item :"+ l_itemNum +":");
			}
				
			l_uttr = replaceQuotes(l_stkn.nextToken());    //3rd column is the utterace
			l_stkn.nextToken(); //Skip the 4th column
			l_tune = replaceQuotes(l_stkn.nextToken()); //5th column is the IntonationIndicatior
			l_response = replaceQuotes(l_stkn.nextToken()); //6th column is the response 	 
			l_list=l_stkn.nextToken(); //7th column is the list Numeric value
			l_stkn.nextToken(); //Skip the 8th column 
			l_stkn.nextToken(); //Skip the 9th column 
			l_stkn.nextToken(); //Skip the 10th column 
			l_stkn.nextToken(); //Skip the 11th column 
			l_scenename=replaceQuotes(l_stkn.nextToken()); //12th column has the pic file name. 
			l_stkn.nextToken(); //Skip the 13th column 
			l_stkn.nextToken(); //Skip the 15th column 
				
			
			//If language is German, then translate the Eng one to German
			if(g_Lang.contains("de")) {
				System.out.print("Eng: "+l_uttr);
				l_uttr=l_Eng2De.get(l_uttr);
				System.out.print(" De: "+l_uttr);	
			}
			
			//So we got the important stuff we need from the CSV file, now process these
			
			//Check if we need to generate soundfiles
			if(b_sndFiles) {
									
				//We have following tunes in the experiment setup:
				/* A : Is that a RED box
				 * B : Is that a red BOX
				 * C : Is THAT a red box (C maps to filler fA )
				 * D : IS that a red box (D maps to filler fB )
				 * The script will generate all 4 by default, and in naming these generated file we
				 * follow the itemNum+tune+Response.wav
				 * 
				 */
				
				//Now convert the plain text utterance to a prosodic-utterance correponding to its tune specified in the csv file
				l_prs = Utterance2Prosodic(l_uttr, l_tuneA );
									
				//Make a RAWMaryXML filename we want to use for Wav files (as discussed it is itemNum+tune)
				l_prs2xml.g_xmlfilename=l_itemNum.concat("_").concat(l_tuneA).concat(".xml");
				
				//Generate RAWMaryXMl for the prosodic-utterance.
				String l_xmlfile = new String();
				l_xmlfile=l_prs2xml.ConvertToRawMarxXml(l_prs);
				//System.out.println("XML file written: ["+l_xmlfile+"]");
				
				//feed this RAWMaryxml to Mary to generate a audio file and save it to disk
				l_ttslocal.m_AudioFileName=l_outputloc_mary.concat(l_prs2xml.g_xmlfilename);
				l_ttslocal.m_SaveAudio2Wav=true;
				l_synthsis.Save2Wave(l_outputloc_mary.concat(l_prs2xml.g_xmlfilename));
				
				//Now repeate this for tune B
				//For B tune
				//Now convert the plain text utterance to a prosodic-utterance correponding to its tune specified in the csv file
				l_prs = Utterance2Prosodic(l_uttr, l_tuneB );
					
				//Make a RAWMaryXML filename we want to use for Wav files (as discussed it is itemNum+tune)
				l_prs2xml.g_xmlfilename=l_itemNum.concat("_").concat(l_tuneB).concat(".xml");
				
				//Generate RAWMaryXMl for the prosodic-utterance.
				l_xmlfile=l_prs2xml.ConvertToRawMarxXml(l_prs);
				//System.out.println("XML file written: ["+l_xmlfile+"]");
								
				//feed this RAWMaryxml to Mary to generate a audio file and save it to disk
				l_ttslocal.m_AudioFileName=l_outputloc_mary.concat(l_prs2xml.g_xmlfilename);
				l_ttslocal.m_SaveAudio2Wav=true;
				l_synthsis.Save2Wave(l_outputloc_mary.concat(l_prs2xml.g_xmlfilename));
				
				
				//For C tune
				//Now convert the plain text utterance to a prosodic-utterance correponding to its tune specified in the csv file
				l_prs = Utterance2Prosodic(l_uttr, l_tuneC );
				String l_tuneC_is_fA = new String("fA");
				//Make a RAWMaryXML filename we want to use for Wav files (as discussed it is itemNum+tune), but for 
				// avoiding confusion to the user, give it a name as per CSV
				l_prs2xml.g_xmlfilename=l_itemNum.concat("_").concat(l_tuneC_is_fA).concat(".xml");
				
				//Generate RAWMaryXMl for the prosodic-utterance.
				l_xmlfile=l_prs2xml.ConvertToRawMarxXml(l_prs);
				//System.out.println("XML file written: ["+l_xmlfile+"]");
			
				//feed this RAWMaryxml to Mary to generate a audio file and save it to disk
				l_ttslocal.m_AudioFileName=l_outputloc_mary.concat(l_prs2xml.g_xmlfilename);
				l_ttslocal.m_SaveAudio2Wav=true;
				l_synthsis.Save2Wave(l_outputloc_mary.concat(l_prs2xml.g_xmlfilename));
				
				//For D tune
				//Now convert the plain text utterance to a prosodic-utterance correponding to its tune specified in the csv file
				l_prs = Utterance2Prosodic(l_uttr, l_tuneD );
				String l_tuneD_is_fB = new String("fB");
				//Make a RAWMaryXML filename we want to use for Wav files (as discussed it is itemNum+tune), but for 
				// avoiding confusion to the user, give it a name as per CSV	
				// Specify the RAWMaryXML filename we want to use here
				l_prs2xml.g_xmlfilename=l_itemNum.concat("_").concat(l_tuneD_is_fB).concat(".xml");
				
				//Generate RAWMaryXMl for the prosodic-utterance.
				l_xmlfile=l_prs2xml.ConvertToRawMarxXml(l_prs);
				//System.out.println("XML file written: ["+l_xmlfile+"]");
				
				//feed this RAWMaryxml to Mary to generate a audio file and save it to disk
				l_ttslocal.m_AudioFileName=l_outputloc_mary.concat(l_prs2xml.g_xmlfilename);
				l_ttslocal.m_SaveAudio2Wav=true;
				l_synthsis.Save2Wave(l_outputloc_mary.concat(l_prs2xml.g_xmlfilename));
			}
			//Sound fine generation is done. For each utterance 4 files (tunes) are generated.
			
			//Now check if WebExp resource list are needed.
			if(b_webExpLst)
			{
				//Needed, so first store all the mappings in some LinkedHashMap,
				// Then use a function to simple generate these file, that is easier, then keeping multiple files open for read/write.
				
				Integer l_resctr=l_ctr-1;
				//Make resource lists as such from the xml file
				//HashMap audio resourse list
				String l_rsrcNum = new String("audio");
				String l_rsrcName = new String("");
				l_rsrcName=l_rsrcName.concat(l_itemNum+"_"+l_tune+"_"+l_response+".wav"); //logic as discussed above.
				l_rsrcNum= l_rsrcNum.concat(l_resctr.toString());
				//System.out.println("Content of: "+"res-audio-exp-list"+l_list+": <"+l_rsrcNum +"> <"+l_rsrcName+">");
			
				//Store the resource Num, and resource Name in a table for respective list
				int list= Integer.parseInt(l_list);
				
				switch(list){
				case 1: l_audioRsrc_1.put(l_rsrcNum, l_rsrcName);
						break;
				case 2: l_audioRsrc_2.put(l_rsrcNum, l_rsrcName);
						break;
				case 3: l_audioRsrc_3.put(l_rsrcNum, l_rsrcName);
						break;
				case 4: l_audioRsrc_4.put(l_rsrcNum, l_rsrcName);
						break;
				case 5: l_audioRsrc_5.put(l_rsrcNum, l_rsrcName);
						break;
				case 6: l_audioRsrc_6.put(l_rsrcNum, l_rsrcName);
						break;
				case 7: l_audioRsrc_7.put(l_rsrcNum, l_rsrcName);
						break;
				case 8: l_audioRsrc_8.put(l_rsrcNum, l_rsrcName);
						break;
				case 0: l_audioRsrc_0.put(l_rsrcNum, l_rsrcName);
				break;
				default :System.out.println("Invalid List.");
					break;
				}
				
				//Store the details for sox script to concatenate Robot and Human utterance.
				//We use the itemcode for a unique name in this HashMap.
				//ItemNum can't help in maintaining this map. 12a_B.wav or f23e_fA.wav is unique
			
				//HashMap robot + human response.
				l_rsrcName=GenAFileName(l_itemcode, l_tune, ".wav");
				l_audioHumRes.put(l_rsrcName, l_response);
				//System.out.println("Content of: sox script: <"+l_rsrcName+"> <"+l_response+">");
							
				//Now store the scene image details.
				//HashMap Image resourse list
				String l_imgNum = new String("image");
				String l_imgName = new String("");
				String l_imgOrient = new String("");
				//Scenes in the CSV have the follwing name:bro-wed_gre-sph, but we have left or right (mirror) orientations
				// to get variety in the scene, and therefore actual jpg files have bro-wed_gre-sph_lef.jpg and bro-wed_gre-sph_rig.jpg
				//names. We call the RandomOrientation to get this left and right in a random fashion.
				l_imgOrient=RandomOrientation(); 
				l_imgName=GenAFileName(l_scenename, l_imgOrient, ".jpg");
				l_imgNum= l_imgNum.concat(l_resctr.toString());
					
				//System.out.println("Content of: res-images-exp-list"+l_list+": <"+l_imgNum+"> <"+l_imgName+">");
				//Store the resource Num, and resource Name in a table for respective list
				switch(list) {
				case 1: l_imageRsrc_1.put(l_imgNum, l_imgName); break;
				case 2: l_imageRsrc_2.put(l_imgNum, l_imgName); break;
				case 3: l_imageRsrc_3.put(l_imgNum, l_imgName); break;
				case 4: l_imageRsrc_4.put(l_imgNum, l_imgName); break;
				case 5: l_imageRsrc_5.put(l_imgNum, l_imgName); break;
				case 6: l_imageRsrc_6.put(l_imgNum, l_imgName); break;
				case 7: l_imageRsrc_7.put(l_imgNum, l_imgName); break;
				case 8: l_imageRsrc_8.put(l_imgNum, l_imgName); break;
				case 0: l_imageRsrc_0.put(l_imgNum, l_imgName); break;
				
				default :System.out.println("Invalid image List.");
				break;
				}
			} //end if WebExp
			//if (l_ctr == 2)break; //Use this break for trial
			
		} //end While read csv, Now close the buffer
		l_CSVbfr.close(); 
		
		//Add all these resources list to 2 main arrays
		l_allImageRes.add(l_imageRsrc_1);
		l_allImageRes.add(l_imageRsrc_2);
		l_allImageRes.add(l_imageRsrc_3);
		l_allImageRes.add(l_imageRsrc_4);
		l_allImageRes.add(l_imageRsrc_5);
		l_allImageRes.add(l_imageRsrc_6);
		l_allImageRes.add(l_imageRsrc_7);
		l_allImageRes.add(l_imageRsrc_8);
		l_allImageRes.add(l_imageRsrc_0);
		
		l_allAudioRes.add(l_audioRsrc_1);
		l_allAudioRes.add(l_audioRsrc_2);
		l_allAudioRes.add(l_audioRsrc_3);
		l_allAudioRes.add(l_audioRsrc_4);
		l_allAudioRes.add(l_audioRsrc_5);
		l_allAudioRes.add(l_audioRsrc_6);
		l_allAudioRes.add(l_audioRsrc_7);
		l_allAudioRes.add(l_audioRsrc_8);
		l_allAudioRes.add(l_audioRsrc_0);
		
		//Now we can simply write the resource files for this experiment.
		if(b_webExpLst) {
			//Write resource files
			
			//File 1: to concatenate a Robot and Human response
			WriteSoxFile();
			
			//WebExp : webExp image and sound resources.
			WriteWebExpResource();
					
		}
		
	}
	catch (Exception e) {
       	e.printStackTrace();
       	System.exit(0);
	}
} //End of Main 	
	
	private static String replaceQuotes(String i_strtkn)
	{
		return i_strtkn.replaceAll("\"", "");
	}
	
	private static String GenAFileName(String i_itemcode, String i_tune, String i_ext ) {
		return i_itemcode.concat("_").concat(i_tune).concat(i_ext);
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
	
	/** Generate a random integer in the range 1..2 */
	
	  private static String RandomOrientation(){
		  String l_tmp= new String("");
		  int randomInt = randomGenerator.nextInt(100);
	     // log("Generated : " + randomInt);
	      
	      if(randomInt % 2 ==0) {
	    	  l_tmp="rig";
	      }else l_tmp="lef";
	    return l_tmp;
	  }
	  
	  private static void WriteSoxFile() {
		  
		  String l_soxfile = new String("mrgRobotHumanRes.sh");
		  //this file will contain a command such as: sox 12_A.wav NO.wav 12_A_NO.wav
		  //sox is a utility on Linux/Unix/Mac for merging wav files.
		  String l_TmpStrKey = new String("");
		  String l_TmpVal = new String("");
		  String l_res = new String("");
		  String l_MaryWavNm = new String("");
		  
		  
		  try {
			  BufferedWriter l_sox_out = new BufferedWriter(new FileWriter(l_outputloc_mary.concat(l_soxfile)));
			
			  Iterator key = l_audioHumRes.keySet().iterator();
			  while(key.hasNext()) {
				  
			  //}
			  //for(Enumeration<String> Key =l_audioHumRes.keys();Key.hasMoreElements();)
			  	//{	
				  	String l_mergecmd = new String("sox");
					//l_TmpStrKey=Key.nextElement();
				  	l_TmpStrKey=(String)key.next();
			  		l_TmpVal=(String)l_audioHumRes.get(l_TmpStrKey);
			  		l_mergecmd=l_mergecmd.concat(" ");
			  		//Table contains key as: 12a_B.wav or f23e_fA.wav (unique key) but for the sox script want to use
			  		// 12_B.wav or 23_fA.wav, SoxMergedFileName returns such file name for these keys
			  		l_MaryWavNm=SoxMergedFileName(l_TmpStrKey);
			  		l_mergecmd=l_mergecmd.concat(l_MaryWavNm);
			  		l_mergecmd=l_mergecmd.concat(" ");
			  		if(l_TmpVal.equals("YES")) {
			  			l_mergecmd=l_mergecmd.concat("YES.wav ");
			  			l_res="YES";
				  	}else {
				  		l_mergecmd=l_mergecmd.concat("NO.wav");
				  		l_res="NO";
				  	}
			  		l_mergecmd=l_mergecmd.concat(" ");
			  		l_MaryWavNm=l_MaryWavNm.replaceAll(".wav", "");
			  		l_mergecmd=l_mergecmd.concat(l_MaryWavNm);
			  		l_mergecmd=l_mergecmd.concat("_");
			  		l_mergecmd=l_mergecmd.concat(l_res);
			  		l_mergecmd=l_mergecmd.concat(".wav");
			  		//System.out.println("merg cmd:-->" + l_mergecmd);
			  		l_sox_out.write(l_mergecmd);
			  		l_sox_out.newLine();
				}
			  l_sox_out.close();
			System.out.println("Sox Merge Script ->"+ l_outputloc_mary.concat(l_soxfile));
		  }
		  catch (Exception fx) {
			System.out.println("Error in writing files: " + fx.toString());
		}
	}
	  
	  private static String SoxMergedFileName(String i_itemcodewav) {
		//i_itemcodewav contains file name as: 12a_B.wav or f23e_fA.wav but for the sox script want to use
	  	// 12_B.wav or 23_fA.wav, SoxMergedFileName returns such file name for these keys
		  
		//file name lenght is the logic 
		String a_tmp = new String("");
		String b_tmp = new String("");
		if(i_itemcodewav.length()==9) {
			a_tmp= i_itemcodewav.substring(0, 2);
			b_tmp= i_itemcodewav.substring(3);
		}else if(i_itemcodewav.length()==11) {
			a_tmp= i_itemcodewav.substring(1, 3);
			b_tmp= i_itemcodewav.substring(4);
		}
		String c_tmp= new String(a_tmp.concat(b_tmp));
		//System.out.println("Sox names: "+ c_tmp);
		return c_tmp;
	  }

	  //No time for this. Casting Object to LinkedHashMap. Let us see suppressing works
	  @SuppressWarnings("unchecked")
	private static void WriteWebExpResource(){
		 
		 int i_list = 0;
		  for (Enumeration<LinkedHashMap> e=l_allImageRes.elements(); e.hasMoreElements();) {
			  String l_imgres = new String("res-images-exp-list");
			  ++i_list;
			  l_imgres=l_imgres.concat(String.valueOf(i_list));
			  l_imgres=l_imgres.concat(".xml");
				
			  LinkedHashMap<String, String> l_tmp = new LinkedHashMap<String, String>();
			  l_tmp = (LinkedHashMap<String, String>)e.nextElement();
			
			  //If the LinkedHashMap has content, continue
			  if(!l_tmp.isEmpty()) {
				  WriteWebExpResource(l_tmp, l_imgres);
				  }
			 }	  
		  
		  i_list=0;
		  for (Enumeration<LinkedHashMap> e=l_allAudioRes.elements(); e.hasMoreElements();) {
			  String l_audres = new String("res-audios-exp-list");
			  ++i_list;
			  l_audres=l_audres.concat(String.valueOf(i_list));
			  //l_audres=l_audres.concat(".xml");
				
			  LinkedHashMap<String, String> l_tmp = new LinkedHashMap<String, String>();
			  l_tmp = (LinkedHashMap<String, String>)e.nextElement();
			  
			  //If the LinkedHashMap has content, continue
			  if(!l_tmp.isEmpty()) {
				  WriteWebExpResource(l_tmp, l_audres);
			  }
		  }	  
		  System.out.println("All Resources written.");
	  }
	  
	  private static void WriteWebExpResource(LinkedHashMap<String, String> i_resName, String i_resourcefile) {
		  
		  //Prepare the xml headers/footer tags for the xml file
		  String l_xmlheader = new String("");
		  l_xmlheader=l_xmlheader.concat("<"+"?"+"xml"+" "+"version"+"="+"\"1.0\""+" "+"encoding"+"="+"\"UTF-8\""+"?"+">");
		 // System.out.println(l_xmlheader);
		  
		  String l_xmlresource = new String("");
		  l_xmlresource=l_xmlresource.concat("<resources>");
		  
		  String l_xmlresourceend = new String("");
		  l_xmlresourceend=l_xmlresourceend.concat("</resources>");
		  
		  String l_xmlblock = new String("");
		  l_xmlblock=l_xmlblock.concat("<block id=\"OneBlock\">");
		  
		  String l_xmlblockend = new String("");
		  l_xmlblockend=l_xmlblockend.concat("<"+"/block"+">");
		
		  
		  try {
			  
			  File l_outputdit = new File(l_outputloc_rsrc);
				boolean l_dir = l_outputdit.exists();
				if(l_dir){
					if(debug){
					System.out.println(" Resource directory exist: " + l_outputloc_rsrc);
					}
				}else{
					if(debug){
					System.out.println(" Resource directory created:" + l_outputloc_rsrc);
					}
					l_outputdit.mkdirs();
				}
				if(debug){
					System.out.println("Writing resource file ");
				}
			 //Resource are being divided in two files as we devide the Experiment into Two Phases
			  BufferedWriter l_resourceFile_ph1 = new BufferedWriter(new FileWriter(l_outputloc_rsrc.concat(i_resourcefile).concat("-phase1.xml")));
			  BufferedWriter l_resourceFile_ph2 = new BufferedWriter(new FileWriter(l_outputloc_rsrc.concat(i_resourcefile).concat("-phase2.xml")));
			  
			  l_resourceFile_ph1.write(l_xmlheader);
			  l_resourceFile_ph1.newLine();
			  l_resourceFile_ph1.newLine();
			  
			  l_resourceFile_ph1.write(l_xmlresource);
			  l_resourceFile_ph1.newLine();
			  l_resourceFile_ph1.newLine();
			  

			  l_resourceFile_ph1.write(l_xmlblock);
			  l_resourceFile_ph1.newLine();
			  l_resourceFile_ph1.newLine();
			  
			  l_resourceFile_ph2.write(l_xmlheader);
			  l_resourceFile_ph2.newLine();
			  l_resourceFile_ph2.newLine();
			  
			  l_resourceFile_ph2.write(l_xmlresource);
			  l_resourceFile_ph2.newLine();
			  l_resourceFile_ph2.newLine();

			  l_resourceFile_ph2.write(l_xmlblock);
			  l_resourceFile_ph2.newLine();
			  l_resourceFile_ph2.newLine();

			  //Now write the resource:
			 Integer l_cntr=0;
			 Iterator e=i_resName.keySet().iterator();
			 while(e.hasNext()) {
				 ++l_cntr;
			 //}
			  //for( Enumeration<String> e = i_resName.keys(); e.hasMoreElements();) {
				  
				  //Here is the actual line in the resource list
				  StringBuffer l_resLine = new StringBuffer("");
				  //String i_imageNum=e.nextElement();
				  String i_imageNum=(String)e.next();
					
				  String i_imageName=i_resName.get(i_imageNum);
				  l_resLine.append("<resource id=\""+i_imageNum+"\">"+i_imageName+"</resource>");
				//  System.out.println("Get from LinkedHashMap: "+l_resLine);
				  if(l_cntr<=48) {
				    l_resourceFile_ph1.write(l_resLine.toString());
				    l_resourceFile_ph1.newLine();
				  }else {
					  l_resourceFile_ph2.write(l_resLine.toString());
					  l_resourceFile_ph2.newLine();
				  }
				  
			  }
			  
			  l_resourceFile_ph1.write(l_xmlblockend);
			  l_resourceFile_ph1.newLine();
			  l_resourceFile_ph1.newLine();
			 
			  l_resourceFile_ph1.write(l_xmlresourceend);
			  l_resourceFile_ph1.newLine();
			  l_resourceFile_ph1.close();
			  
			  l_resourceFile_ph2.write(l_xmlblockend);
			  l_resourceFile_ph2.newLine();
			  l_resourceFile_ph2.newLine();
			 
			  l_resourceFile_ph2.write(l_xmlresourceend);
			  l_resourceFile_ph2.newLine();
			  l_resourceFile_ph2.close();
			
			System.out.println("Resource written ->"+ l_outputloc_rsrc.concat(i_resourcefile).concat("-phase1.xml"));
			System.out.println("Resource written ->"+ l_outputloc_rsrc.concat(i_resourcefile).concat("-phase2.xml"));
		  }
		  catch (Exception fx) {
			System.out.println("Error in writing files: " + fx.toString());
		}
	  }
} //class
