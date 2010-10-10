// =================================================================
// Copyright (C) 2009-2010 DFKI GmbH Talking Robots
// Raveesh Meena (rame01@dfki.de)
//
// This library is free software; you can redistribute it and/or
// modify it under the terms of the GNU Lesser General Public License
// as published by the Free Software Foundation; either version 2.1 of
// the License, or (at your option) any later version.
//
// This library is distributed in the hope that it will be useful, but
// WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
// Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public
// License along with this program; if not, write to the Free Software
// Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA
// 02111-1307, USA.
// =================================================================

// =================================================================
// PACKAGE DEFINITION
package de.dfki.lt.tr.dialogue.tts;

import java.io.*;

import java.sql.Timestamp;
import java.util.*;

import marytts.client.MaryClient;


public class ProsodicTextToRawMARYXml {

	private static final String XMLtag_whitespace = new String(" ");
	private static final String XMLtag_hyphen = new String("-");
	private static final String XMLtag_ACCENT_open = new String("<t");
	private static final String XMLtag_ACCENT_prefix = new String("accent=\"");
	private static final String XMLtag_ACCENT_suffix = new String("\">");
	private static final String XMLtag_ACCENT_close = new String("</t>");
	
	private static final String XMLtag_BOUNDARY_open = new String("<boundary");
	private static final String XMLtag_TONE_prefix = new String("tone=\"");
	private static final String XMLtag_TONE_suffix = new String("\"");
	private static final String XMLtag_BREAKInd_prefix = new String("breakindex=\"");
	private static final String XMLtag_BREAKInd_suffix = new String("\"");
	private static final String XMLtag_BOUNDARY_close = new String("/>");
	private static String XMLtag_BOUNDARY_pause = new String("6");
	
	private static final String SententialPause = new String ("6");
	private static final String IntraSententialPause = new String ("2");
	private static final String ProsodicKey = new String ("@");
	private static final String BoundryKey = new String ("%");
	
	private  String RAWMARYXMLHead;
	public   String g_xmlfilename=null;
	private  String GenratedXMLFileLocation;
	private  Integer UtteranceCount=1;
	private  String g_stub;
			
	private boolean debug=false;
	/**
	 * @param i_maryXmlHdr RAWMaryXml header.
	 * @param i_writeFile2Dir Location to keep the generated XMLFile.
	 * @param i_stub  a prefix for XMLFilename.
	 * @param i_prosody text is prosodic.
	 */
	public ProsodicTextToRawMARYXml(String i_maryXmlHdr, String i_writeFile2Dir, String i_stub){
		this.RAWMARYXMLHead=i_maryXmlHdr;
		this.GenratedXMLFileLocation=i_writeFile2Dir;
		this.g_stub=i_stub;
	}
	/**	A function that takes the prosodic text as input, converts it into RawMaryXML and returns the filename 
	 * @param inp_prosodictxt text to be spoken.
	 */
	//This is a function called from cc_TTS / or other Components requesting Speech Synthesis
	// Any changes in main() should also be reflected here
	public  String ConvertToRawMarxXml(String i_prosodictxt){
		if(debug){
			System.out.println("Convert to XML got: " + i_prosodictxt);
		}
		
		StringBuffer l_str2xml =new StringBuffer();
		//tokenize the input string
		StringTokenizer l_tkns = new StringTokenizer(i_prosodictxt," ");
		Integer l_tkncnt=l_tkns.countTokens();
		Integer l_bndrycnt=0;
		while (l_tkns.hasMoreTokens()){
			++l_bndrycnt;
			String l_word=l_tkns.nextToken();
			String l_returned_xml =new String("");;
			
			//token bears accent
			if(AccentedWord(l_word)) {
			l_returned_xml=HandleAccentedWord(l_word);
			l_str2xml.append(l_returned_xml);
			l_str2xml.append(XMLtag_whitespace);
			}
			else if(isBoundaryTone(l_word)){
			//token is boundary tone
			//check if it is a intra-sentential boundary
			//System.out.println("boundary:" + l_bndrycnt + "tkn cnt:" + l_tkncnt);
			if(l_bndrycnt < l_tkncnt) XMLtag_BOUNDARY_pause=IntraSententialPause;
			else XMLtag_BOUNDARY_pause=SententialPause;
			
			l_returned_xml=HandleBoundaryTone(l_word);
			l_str2xml.append(l_returned_xml);
			l_str2xml.append(XMLtag_whitespace);
			}
			else {
				l_returned_xml=HandleUnAccentedWord(l_word);
				l_str2xml.append(l_returned_xml);
				l_str2xml.append(XMLtag_whitespace);
			}
		}
	
		//Put this string in the XMLfile (containing some fixed header and end tags) and return the filename.
		return this.WriteToXML(l_str2xml.toString());
	}

	private static boolean AccentedWord(String i_word){
		if (i_word.contains(ProsodicKey)) return true; //for the moment we only check for an "@" in accented words. e.g. red_H*, red_L+H*
		else return false;
		
	}
	private static boolean isBoundaryTone(String i_btone){
		if(i_btone.contains(BoundryKey)) return true; //for the moment we only check for an "%" in boundary tones. e.g. LL%, LH%, HH%, HL%.
		else return false;
	}
	
	private String HandleAccentedWord(String i_acc_word) {
		if(debug){
			System.out.println("Handle accented word got: " + i_acc_word);
		}
		
		StringBuffer l_xml2rtrn =new StringBuffer();
		l_xml2rtrn.append(XMLtag_ACCENT_open);
		l_xml2rtrn.append(XMLtag_whitespace);
		l_xml2rtrn.append(XMLtag_ACCENT_prefix);
		//Tokenise word and accent, i.e. red_H* into red and H*
		StringTokenizer l_tkns = new StringTokenizer(i_acc_word,ProsodicKey);
		String l_word =new String("");
		String l_accent = new String("");
		while (l_tkns.hasMoreTokens()){
			
			l_word=l_tkns.nextToken();
			l_accent=l_tkns.nextToken();
		}
		l_xml2rtrn.append(l_accent);
		l_xml2rtrn.append(XMLtag_ACCENT_suffix);
		l_xml2rtrn.append(l_word);
		l_xml2rtrn.append(XMLtag_ACCENT_close);
				
		return l_xml2rtrn.toString();

	}
	private String HandleUnAccentedWord(String i_acc_word) {
		if(debug){
			System.out.println("Handle accented word got: " + i_acc_word);
		}
		
		StringBuffer l_xml2rtrn =new StringBuffer();
		l_xml2rtrn.append(XMLtag_ACCENT_open);
		l_xml2rtrn.append(XMLtag_whitespace);
		l_xml2rtrn.append(XMLtag_ACCENT_prefix);
		l_xml2rtrn.append("none");
		l_xml2rtrn.append(XMLtag_ACCENT_suffix);
		l_xml2rtrn.append(i_acc_word);
		l_xml2rtrn.append(XMLtag_ACCENT_close);
				
		return l_xml2rtrn.toString();

	}
	private String HandleBoundaryTone(String i_bndry){
		if(debug){
			System.out.println("Handle boundary tones got: " + i_bndry);
		}
		String l_lhs =new String("");
		String l_rhs = new String("");;
		StringBuffer l_xml2rtrn =new StringBuffer();
		l_xml2rtrn.append(XMLtag_BOUNDARY_open);
		l_xml2rtrn.append(XMLtag_whitespace);
		l_xml2rtrn.append(XMLtag_TONE_prefix);
		
		 //RawMaryXML takes LL% as L-L%, !LH% as !L-H% and L as L-
		 if(i_bndry.length()>2 && i_bndry.length() <5 ){
			//e.g. LL% as L-L%, !LH% as !L-H%
			l_lhs=i_bndry.substring(0,1);
			l_rhs=i_bndry.substring(1);
		 }else if (i_bndry.length()<=2){
			//e.g. !H as !H- , H as H- and so on
			l_lhs=i_bndry;
			l_rhs="";
		 }else if (i_bndry.length()==5){
			l_lhs=i_bndry.substring(0,2);
			l_rhs=i_bndry.substring(2);
		 }
				
		 l_xml2rtrn.append(l_lhs);
		 l_xml2rtrn.append(XMLtag_hyphen);
		 l_xml2rtrn.append(l_rhs);
		 l_xml2rtrn.append(XMLtag_TONE_suffix);
		 l_xml2rtrn.append(XMLtag_whitespace);
		 l_xml2rtrn.append(XMLtag_BREAKInd_prefix);
		 l_xml2rtrn.append(XMLtag_BOUNDARY_pause);
		 l_xml2rtrn.append(XMLtag_BREAKInd_suffix);
		 l_xml2rtrn.append(XMLtag_BOUNDARY_close);
		  
		return l_xml2rtrn.toString();
	}
	
	public String XmlFileName(final String i_prsdstr){
		
		//convert a prosodic string into a filename
		String l_str2fileNm = new String(i_prsdstr);
		l_str2fileNm=l_str2fileNm.replace(" ", "");
		l_str2fileNm=l_str2fileNm.replace("@", "");
		l_str2fileNm=l_str2fileNm.replace("*", "s");
		l_str2fileNm=l_str2fileNm.replace("+", "p");
		l_str2fileNm=l_str2fileNm.replace("%", "");
		
		
		StringBuffer l_xmlfilename = new StringBuffer();
		l_xmlfilename.append(UtteranceCount.toString());
		l_xmlfilename.append(g_stub);
		l_xmlfilename.append("_rwMary_");
		
		Date l_date = new Date();
		Timestamp l_timestamp = new Timestamp(l_date.getTime());
		String l_datetime = new String(l_timestamp.toString());
		l_datetime=l_datetime.replace(":", "");
		l_datetime=l_datetime.replace(".", "");
		l_datetime=l_datetime.replace("-", "");
		l_datetime=l_datetime.replace(" ", "");
		
		l_xmlfilename.append(l_str2fileNm);
		l_xmlfilename.append("_");
		l_xmlfilename.append(l_datetime);
		l_xmlfilename.append(".xml");
			
		return l_xmlfilename.toString();
	}
	
	private String WriteToXML(String i_xmlstring){
		if(debug){
			System.out.println("WriteToXML got: " + i_xmlstring);
			System.out.println("XML Head is: " + this.RAWMARYXMLHead);
		}
		
		//XmlFile on the file system
		String l_xmlfile=new String(GenratedXMLFileLocation.concat(g_xmlfilename));
		
		//Prepare the header section of MaryXML file
		StringBuffer l_rawxml_final = new StringBuffer();
		try {
			BufferedReader l_head = new BufferedReader(new FileReader(RAWMARYXMLHead));
			String st;
			while ((st=l_head.readLine()) != null) {
				l_rawxml_final.append(st);
				l_rawxml_final.append(XMLtag_whitespace);
			}
			l_head.close();
			}
		catch (Exception fx) {
			System.out.println("IO error in reading RAWMARYXMLHead" + fx.toString());
		}
		
		//append our XMLstring to this header
		l_rawxml_final.append(i_xmlstring);
		//close the header section of MaryXML file
		l_rawxml_final.append("</maryxml>");
		
		//Now write this all to a XML file 
			
		if(debug){
			System.out.println("Writing file to location: " + GenratedXMLFileLocation);
		}
		try {
			File l_outputdit = new File(GenratedXMLFileLocation);
			boolean l_dir = l_outputdit.exists();
			if(l_dir){
				if(debug){
				System.out.println(" Write2XML directory exist: " + GenratedXMLFileLocation);
				}
			}else{
				if(debug){
				System.out.println(" Write2XML directory created:" + GenratedXMLFileLocation);
				}
				l_outputdit.mkdirs();
			}
			if(debug){
				System.out.println("Writing file ");
			}
			
			BufferedWriter l_xml_out = new BufferedWriter(new FileWriter(l_xmlfile));
			l_xml_out.write(l_rawxml_final.toString());
			l_xml_out.close();
			}
		catch (Exception fx) {
			System.out.println("Error in Write2XML: " + fx.toString());
		}
		return l_xmlfile;
	}
	/**
	 * @param args RAWMARYXml header, location to keep the generated XMLFile 
	 */
	
public static void main(String[] args) {
	// TODO Auto-generated method stub
	
	String l_xmlfile = new String();
	String l_voicename = new String();
	Integer l_substr;
	//Initialise an Instant of TTS Local.
	try{
		MaryClient l_mary = MaryClient.getMaryClient();
		MaryTTSLocal l_ttslocal = new MaryTTSLocal(l_mary,"RAWMARYXML", "us2", false, "WAVE");
		
		System.out.println("RAWMARYXMLHead is in file : " + args[1]);
		ProsodicTextToRawMARYXml l_convert = new ProsodicTextToRawMARYXml(args[1], args[2],"");
		SynthesisRAWMaryXMLInput l_synth = new SynthesisRAWMaryXMLInput(l_ttslocal);
		
		
		//Get the prosodic utterance text from a file
		try{
		FileReader l_file = new FileReader(args[0]); 
		BufferedReader l_inp= new BufferedReader(l_file);  
		String l_utterance = new String();
		String l_stub = new String();
		
		//read a line
		while((l_utterance=l_inp.readLine())!=null){
			
			System.out.println("I read: " + l_utterance);
			
			if(l_utterance.startsWith("#")){
				continue;
			}
			if(l_utterance.startsWith("h")){
				l_voicename="hmm-jmk";
				l_substr=2;
				l_stub="h";
			}else if(l_utterance.startsWith("r")){
				l_voicename="us2";
				l_substr=2;
				l_stub="r";
			}else {
				l_voicename="us2"; 
				l_substr=0;
				l_stub="g";
			}
						
			//Just an indicator
		l_convert.g_stub=l_stub;
			
		//Make a user friendly filename:
		l_convert.g_xmlfilename = new String("");
		l_convert.g_xmlfilename= l_convert.XmlFileName(l_utterance.substring(l_substr));
		//A function that takes the prosodic text as input, converts it into RawMaryXML and returns the filename 
		l_xmlfile=l_convert.ConvertToRawMarxXml(l_utterance.substring(l_substr));
		//Keep the trace of line count
		l_convert.UtteranceCount++;
		
		System.out.println("XML file written: "+ l_xmlfile);
			
		//Synthesize this file
		l_synth.m_ttsLocal.m_AudioFileName=l_xmlfile;
		l_synth.m_ttsLocal.m_SaveAudio2Wav=Boolean.valueOf(args[3]);
		l_synth.m_ttsLocal.m_voiceName=l_voicename;
		l_synth.Utter(l_convert.GenratedXMLFileLocation.concat(l_convert.g_xmlfilename));
		
		if(!l_synth.m_ttsLocal.m_SaveAudio2Wav){
			File f = new File(l_convert.GenratedXMLFileLocation.concat(l_convert.g_xmlfilename));
			if(f.exists()){
				System.out.println("XML file deleted:"+ l_convert.GenratedXMLFileLocation.concat(l_convert.g_xmlfilename));
				boolean success = f.delete();
			    if (!success)
			     throw new IllegalArgumentException("Delete RamMaryXMl failed");
			}
		}
		Thread.sleep(2500);
		}
	}
	catch (IOException io_e) {
		// TODO: handle exception
		System.out.println(io_e);
	}
	}catch (Exception e) {
		// TODO Auto-generated catch block
		e.printStackTrace();
	}
	
	//Delete the generated RAWMaryXML
	
	}
}
