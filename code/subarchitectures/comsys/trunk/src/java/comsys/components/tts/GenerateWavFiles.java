package comsys.components.tts;

import java.io.BufferedReader;
import java.io.BufferedWriter;
import java.io.FileReader;
import java.io.FileWriter;
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
	public static Hashtable<String, String> l_audioRsrc_1 = new Hashtable<String, String>();
	public static Hashtable<String, String> l_audioRsrc_2 = new Hashtable<String, String>();
	public static Hashtable<String, String> l_audioRsrc_3 = new Hashtable<String, String>();
	public static Hashtable<String, String> l_audioRsrc_4 = new Hashtable<String, String>();
	public static Hashtable<String, String> l_audioRsrc_5 = new Hashtable<String, String>();
	public static Hashtable<String, String> l_audioRsrc_6 = new Hashtable<String, String>();
	public static Hashtable<String, String> l_audioRsrc_7 = new Hashtable<String, String>();
	public static Hashtable<String, String> l_audioRsrc_8 = new Hashtable<String, String>();
	
	public static Hashtable<String, String> l_imageRsrc_1 = new Hashtable<String, String>();
	public static Hashtable<String, String> l_imageRsrc_2 = new Hashtable<String, String>();
	public static Hashtable<String, String> l_imageRsrc_3 = new Hashtable<String, String>();
	public static Hashtable<String, String> l_imageRsrc_4 = new Hashtable<String, String>();
	public static Hashtable<String, String> l_imageRsrc_5 = new Hashtable<String, String>();
	public static Hashtable<String, String> l_imageRsrc_6 = new Hashtable<String, String>();
	public static Hashtable<String, String> l_imageRsrc_7 = new Hashtable<String, String>();
	public static Hashtable<String, String> l_imageRsrc_8 = new Hashtable<String, String>();
	
	public static Hashtable<String, String> l_audioHumRes = new Hashtable<String, String>();
	
	private static Random randomGenerator = new Random();
	private static String l_outputloc;
	private static String l_outputloc_mary;
	private static String l_outputloc_rsrc;
	/**
	 * @param args
	 */
	public static void main(String[] args) {
		// TODO Auto-generated method stub
		String l_rawmary = new String("./subarchitectures/comsys/src/java/comsys/components/tts/RAWMARYXMLhead.xml");
		
		try{
		FileReader l_xlsreader = new FileReader(args[0]);
		l_outputloc = new String(args[1]);
		l_outputloc_mary = new String(l_outputloc.concat("/maryGen/"));
		l_outputloc_rsrc = new String(l_outputloc.concat("/rswebExp/"));
		
		ProsodicTextToRawMARYXml l_prs2xml = new ProsodicTextToRawMARYXml(l_rawmary,l_outputloc_mary,"misc");
		
		
		MaryClient l_mary = new MaryClient("localhost", 59125);
		TTSLocal l_ttslocal = new TTSLocal(l_mary,"RAWMARYXML", "us2", false, "WAVE");
		SynthesisRAWMaryXMLInput l_synthsis = new SynthesisRAWMaryXMLInput(l_ttslocal);
		//Read the xls file
		
		//we need utterance, intonation type A, B, C, D, and file-name to save.
		
		boolean b_sndFiles =false;
		boolean b_webExpLst=false;
		
		if(args[2].equals("WavFiles")) {
			b_sndFiles=true;
		}
		if(args[3].equals("WebExp")) {
			b_webExpLst=true;
		}
			
		BufferedReader l_xlsbfr = new BufferedReader(l_xlsreader);	
		String l_str = null;
		Integer l_ctr= 0;
		while((l_str= l_xlsbfr.readLine())!= null && l_ctr<=48){
			++l_ctr;
			if(l_ctr==1) continue; //First line is the table header, skip it
				
				
			System.out.println(l_ctr + ":"+ l_str);
			String l_list= new String("");
			String l_itemcode = new String("");
			String l_itemNum = new String("");
			
			String l_uttr = new String("");
			String l_tune = new String("");
			String l_response = new String("");
			
			String l_scenename = new String("");
			String l_prs = new String("");
			StringTokenizer l_stkn = new StringTokenizer(l_str, ",");
				
			l_itemcode= replaceQuotes(l_stkn.nextToken()); //Skip the 1st column
			
			l_itemNum=l_itemcode.substring(0, 2);
			//System.out.println("item :"+ l_itemNum +":");
				
			l_uttr = replaceQuotes(l_stkn.nextToken());    //2nd column is the utterace
			l_stkn.nextToken(); //Skip the 3rd column
			l_tune = replaceQuotes(l_stkn.nextToken()); //4th column is the IntonationIndicatior
			l_response = replaceQuotes(l_stkn.nextToken()); //5th column is the response 	 
			l_list=replaceQuotes(l_stkn.nextToken()); //6th column is the list 
			l_stkn.nextToken(); //Skip the 7th column 
			l_stkn.nextToken(); //Skip the 8th column 
			l_stkn.nextToken(); //Skip the 9th column 
			l_stkn.nextToken(); //Skip the 10th column 
			l_scenename=replaceQuotes(l_stkn.nextToken()); //11th column has the pic file name. 
			l_stkn.nextToken(); //Skip the 12th column 
			l_stkn.nextToken(); //Skip the 13th column 
				
			//We got the important stuff we needed
			
			//See if we need to generate soundfiles
			if(b_sndFiles) {
									
				//For A tune
				//Now convert the utterance to prosodic-utterance with the help of Intonation contour type.
				l_prs = Utterance2Prosodic(l_uttr, l_tuneA );
									
				// Specify the RAWMaryXML filename we want to use here
				l_prs2xml.g_xmlfilename=l_itemNum.concat("_").concat(l_tuneA).concat(".xml");
				//l_prs2xml.g_xmlfilename=GenAFileName(l_itemNum, l_tuneA, ".xml");
				
				//converts prosodic-utterance into RawMaryXML with above filename
				String l_xmlfile = new String();
				l_xmlfile=l_prs2xml.ConvertToRawMarxXml(l_prs);
				System.out.println("XML file written: ["+l_xmlfile+"]");
				
				// use Mary to generate a audio file and save it to disk
				l_ttslocal.m_AudioFileName=l_outputloc_mary.concat(l_prs2xml.g_xmlfilename);
				l_ttslocal.m_SaveAudio2Wav=true;
				l_synthsis.Save2Wave(l_outputloc_mary.concat(l_prs2xml.g_xmlfilename));
				
				//For B tune
				//Now convert the utterance to prosodic-utterance with the help of Intonation contour type.
				l_prs = Utterance2Prosodic(l_uttr, l_tuneB );
					
				// Specify the RAWMaryXML filename we want to use here
				l_prs2xml.g_xmlfilename=l_itemNum.concat("_").concat(l_tuneB).concat(".xml");
				
				//converts prosodic-utterance into RawMaryXML
				l_xmlfile=l_prs2xml.ConvertToRawMarxXml(l_prs);
				System.out.println("XML file written: ["+l_xmlfile+"]");
				
				
				// use Mary to generate a audio file and save it to disk
				l_ttslocal.m_AudioFileName=l_outputloc_mary.concat(l_prs2xml.g_xmlfilename);
				l_ttslocal.m_SaveAudio2Wav=true;
				l_synthsis.Save2Wave(l_outputloc_mary.concat(l_prs2xml.g_xmlfilename));
				
				
				//For C tune
				//Now convert the utterance to prosodic-utterance with the help of Intonation contour type.
				l_prs = Utterance2Prosodic(l_uttr, l_tuneC );
									
				// Specify the RAWMaryXML filename we want to use here
				l_prs2xml.g_xmlfilename=l_itemNum.concat("_").concat(l_tuneC).concat(".xml");
				
				//converts prosodic-utterance into RawMaryXML
				l_xmlfile=l_prs2xml.ConvertToRawMarxXml(l_prs);
				System.out.println("XML file written: ["+l_xmlfile+"]");
			
				// use Mary to generate a audio file and save it to disk
				l_ttslocal.m_AudioFileName=l_outputloc_mary.concat(l_prs2xml.g_xmlfilename);
				l_ttslocal.m_SaveAudio2Wav=true;
				l_synthsis.Save2Wave(l_outputloc_mary.concat(l_prs2xml.g_xmlfilename));
				
				//For B tune
				//Now convert the utterance to prosodic-utterance with the help of Intonation contour type.
				l_prs = Utterance2Prosodic(l_uttr, l_tuneD );
					
				// Specify the RAWMaryXML filename we want to use here
				l_prs2xml.g_xmlfilename=l_itemNum.concat("_").concat(l_tuneD).concat(".xml");
				
				//converts prosodic-utterance into RawMaryXML
				l_xmlfile=l_prs2xml.ConvertToRawMarxXml(l_prs);
				System.out.println("XML file written: ["+l_xmlfile+"]");
				
				// use Mary to generate a audio file and save it to disk
				l_ttslocal.m_AudioFileName=l_outputloc_mary.concat(l_prs2xml.g_xmlfilename);
				l_ttslocal.m_SaveAudio2Wav=true;
				l_synthsis.Save2Wave(l_outputloc_mary.concat(l_prs2xml.g_xmlfilename));
			}
									
			if(b_webExpLst)
			{
				Integer l_resctr=l_ctr-1;
				//Make resource lists as such from the xml file
				//HashMap audio resourse list
				String l_rsrcNum = new String("audio");
				String l_rsrcName = new String("");
				l_rsrcName=GenAFileName(l_itemNum, l_tune, ".wav");
				l_rsrcNum= l_rsrcNum.concat(l_resctr.toString());
			
				System.out.println("audiores_"+l_list+":"+l_rsrcNum +"--"+l_rsrcName);
				
				switch((int)(Integer.valueOf(l_list))) {
				case 1: l_audioRsrc_1.put(l_rsrcNum, l_rsrcName);
				case 2: l_audioRsrc_1.put(l_rsrcNum, l_rsrcName);
				case 3: l_audioRsrc_1.put(l_rsrcNum, l_rsrcName);
				case 4: l_audioRsrc_1.put(l_rsrcNum, l_rsrcName);
				case 5: l_audioRsrc_1.put(l_rsrcNum, l_rsrcName);
				case 6: l_audioRsrc_1.put(l_rsrcNum, l_rsrcName);
				case 7: l_audioRsrc_1.put(l_rsrcNum, l_rsrcName);
				case 8: l_audioRsrc_1.put(l_rsrcNum, l_rsrcName);
				default :System.out.println("Invalid List.");
				break;
				}
				System.out.println("audiores:"+l_itemcode+":"+l_rsrcName);
				//HashMap robot + human response.
				l_rsrcName=GenAFileName(l_itemcode, l_tune, ".wav");
				l_audioHumRes.put(l_rsrcName, l_response);
				System.out.println("robotHuam:"+l_rsrcName+":"+l_response);
				
					
				//HashMap audio resourse list
				String l_imgNum = new String("image");
				String l_imgName = new String("");
				String l_imgSfx = new String("");
				l_imgSfx=RandomOrientation();
				l_imgName=GenAFileName(l_scenename, l_imgSfx, ".jpg");
				l_imgNum= l_imgNum.concat(l_resctr.toString());
					
				System.out.println("imageres_"+l_list+":"+l_imgNum+"--"+l_imgName);
				
				switch((int)(Integer.valueOf(l_list))) {
				case 1: l_imageRsrc_1.put(l_imgNum, l_imgName);
				case 2: l_imageRsrc_2.put(l_imgNum, l_imgName);
				case 3: l_imageRsrc_3.put(l_imgNum, l_imgName);
				case 4: l_imageRsrc_4.put(l_imgNum, l_imgName);
				case 5: l_imageRsrc_5.put(l_imgNum, l_imgName);
				case 6: l_imageRsrc_6.put(l_imgNum, l_imgName);
				case 7: l_imageRsrc_7.put(l_imgNum, l_imgName);
				case 8: l_imageRsrc_8.put(l_imgNum, l_imgName);
				default :System.out.println("Invalid image List.");
				break;
				}
			} //end if WebExp
			if (l_ctr == 15)break;
		} //end While read xsl
		
		if(b_webExpLst) {
			//Write resource files
			//File to concatenate a Robot and Human response
			WriteSoxFile();
			
			
		}
		
	}
	catch (Exception e) {
       	e.printStackTrace();
	}
} //main
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
	      log("Generated : " + randomInt);
	      
	      if(randomInt % 2 ==0) {
	    	  l_tmp="rig";
	      }else l_tmp="lef";
	    return l_tmp;
	  }
	  
	  private static void log(String aMessage){
	    System.out.println(aMessage);
	  }

	  private static void WriteSoxFile() {
		  
		  String l_soxfile = new String("mrgRobotHumanRes.sh");
		  String l_TmpStrKey = new String("");
		  String l_TmpVal = new String("");
		  String l_res = new String("");
		  String l_MaryWavNm = new String("");
		  
		  
		  try {
			  BufferedWriter l_sox_out = new BufferedWriter(new FileWriter(l_outputloc_rsrc.concat(l_soxfile)));
			
			  for(Enumeration<String> Key =l_audioHumRes.keys();Key.hasMoreElements();)
			  	{	
				  	String l_mergecmd = new String("sox");
					l_TmpStrKey=Key.nextElement();
			  		l_TmpVal=l_audioHumRes.get(l_TmpStrKey);
			  		l_mergecmd=l_mergecmd.concat(" ");
			  		l_MaryWavNm=GeneratedFileName(l_TmpStrKey);
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
			  		
			  		l_sox_out.write(l_mergecmd);
			  		l_sox_out.newLine();
				}
			  l_sox_out.close();
				
		  }
	  catch (Exception fx) {
			System.out.println("Error in Write2XML: " + fx.toString());
		}
		
	  }
	  
	  private static String GeneratedFileName(String i_itemcodewav) {
		  String a_tmp = new String(i_itemcodewav.substring(0, 2));
		  String b_tmp = new String(i_itemcodewav.substring(3));
		  String c_tmp= new String(a_tmp.concat(b_tmp));
		  System.out.println("Sox names: "+ c_tmp);
		  return c_tmp;
	  }
}
