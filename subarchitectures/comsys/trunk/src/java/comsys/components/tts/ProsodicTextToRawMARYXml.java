package comsys.components.tts;

import java.io.*;

import java.sql.Timestamp;
import java.util.*;

public class ProsodicTextToRawMARYXml {

	private static boolean debug=false;
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
	private static String XMLtag_BOUNDARY_pause = new String("4");
	
	private static String RAWMARYXMLHead;
	private static String GenratedXMLFileLocation;
	private static Integer UtteranceCount=1;
	private static String stub;
	private static boolean prosody=false;
	
	private static final String SententialPause = new String ("4");
	private static final String IntraSententialPause = new String ("2");
	/**
	 * @param args
	 */
	public static void main(String[] args) {
		// TODO Auto-generated method stub
		
		//Save some global variables
		GenratedXMLFileLocation = new String(args[2]);
		RAWMARYXMLHead = new String(args[1]);
		UtteranceCount=0;
		String l_xmlfile = new String();
		String l_voicename = new String();
		Integer l_substr;
		if(debug){
			System.out.println("RAWMARYXMLHead is in file : " + RAWMARYXMLHead);
		}
		//Get the prosodic utterance text from a file
		try{
			FileReader l_file = new FileReader(args[0]); 
			BufferedReader l_inp= new BufferedReader(l_file);  
			String l_utterance = new String();
			
			//read a line
			while((l_utterance=l_inp.readLine())!=null){
				if(debug){
					System.out.println("I read: " + l_utterance);
				}
				if(l_utterance.startsWith("#")){
					continue;
				}
				if(l_utterance.startsWith("h")){
					l_voicename="hmm-jmk";
					l_substr=2;
					stub="h";
				}else if(l_utterance.startsWith("r")){
					l_voicename="us2";
					l_substr=2;
					stub="r";
				}else {
					l_voicename="us2"; 
					l_substr=0;
					stub="g";
				}
				
				
			//Keep the trace of line count
				UtteranceCount++;
			//Just an indicator
				if(l_utterance.contains("%") || l_utterance.contains("_")) prosody= true;
				else prosody= false;
			//pass this line to the conversion function
				
				l_xmlfile=ConvertToRawMarxXml(l_utterance.substring(l_substr));
				
				//A function that takes the prosodic text as input and returns a filename
				System.out.println("XML file written: "+ l_xmlfile);
				
				//Synthesize this file
				try {
						SynthesisRAWMaryXMLInput.Utter(GenratedXMLFileLocation.concat(l_xmlfile),l_voicename);
						Thread.sleep(2500);
				} catch (Exception e) {
					// TODO Auto-generated catch block
					e.printStackTrace();
				}
				
			}
			
			
		}
		catch (IOException io_e) {
			// TODO: handle exception
			System.out.println(io_e);
		}
		
		
		
	}
	public static String ConvertToRawMarxXml(String inp_prosodictxt){
		if(debug){
			System.out.println("Convert to XML got: " + inp_prosodictxt);
		}
		
		StringBuffer l_xmlstring =new StringBuffer();
		String l_xmlfile_return = new String();
		//tokenize the input string
		StringTokenizer l_tkns = new StringTokenizer(inp_prosodictxt," ");
		Integer l_tkncnt=l_tkns.countTokens();
		Integer l_bndrycnt=0;
		while (l_tkns.hasMoreTokens()){
			++l_bndrycnt;
			String l_word=l_tkns.nextToken();
			String l_returned_xml =new String("");;
			
			//token bears accent
			if(AccentedWord(l_word)) {
			l_returned_xml=HandleAccentedWord(l_word);
			l_xmlstring.append(l_returned_xml);
			l_xmlstring.append(XMLtag_whitespace);
			}
			else if(isBoundaryTone(l_word)){
			//token is boundary tone
			//check if it is a intra-sentential boundary
			//System.out.println("boundary:" + l_bndrycnt + "tkn cnt:" + l_tkncnt);
			if(l_bndrycnt < l_tkncnt) XMLtag_BOUNDARY_pause=IntraSententialPause;
			else XMLtag_BOUNDARY_pause=SententialPause;
			
			l_returned_xml=HandleBoundaryTone(l_word);
			l_xmlstring.append(l_returned_xml);
			l_xmlstring.append(XMLtag_whitespace);
			}
			else {
				l_returned_xml=HandleUnAccentedWord(l_word);
				l_xmlstring.append(l_returned_xml);
				l_xmlstring.append(XMLtag_whitespace);
			}
		}
		//Put this string in the XMLfile that has fixed tags
		l_xmlfile_return=WriteToXML(l_xmlstring.toString());
		
		return l_xmlfile_return;
	}

	public static boolean AccentedWord(String i_word){
		if (i_word.contains("_")) return true; //for the moment we only check for an "_" in accented words. e.g. red_H*, red_L+H*
		else return false;
		
	}
	public static boolean isBoundaryTone(String i_btone){
		if(i_btone.contains("%")) return true; //for the moment we only check for an "%" in boundary tones. e.g. LL%, LH%, HH%, HL%.
		else return false;
	}
	public static String HandleAccentedWord(String i_acc_word) {
		if(debug){
			System.out.println("Handle accented word got: " + i_acc_word);
		}
		
		StringBuffer l_xmlto_return =new StringBuffer();
		l_xmlto_return.append(XMLtag_ACCENT_open);
		l_xmlto_return.append(XMLtag_whitespace);
		l_xmlto_return.append(XMLtag_ACCENT_prefix);
		//Tokenise word and accent, i.e. red_H* into red and H*
		StringTokenizer l_tkns = new StringTokenizer(i_acc_word,"_");
		String l_word =new String("");
		String l_accent = new String("");
		while (l_tkns.hasMoreTokens()){
			
			l_word=l_tkns.nextToken();
			l_accent=l_tkns.nextToken();
		}
		l_xmlto_return.append(l_accent);
		l_xmlto_return.append(XMLtag_ACCENT_suffix);
		l_xmlto_return.append(l_word);
		l_xmlto_return.append(XMLtag_ACCENT_close);
				
		return l_xmlto_return.toString();

	}
	public static String HandleUnAccentedWord(String i_acc_word) {
		if(debug){
			System.out.println("Handle accented word got: " + i_acc_word);
		}
		
		StringBuffer l_xmlto_return =new StringBuffer();
		l_xmlto_return.append(XMLtag_ACCENT_open);
		l_xmlto_return.append(XMLtag_whitespace);
		l_xmlto_return.append(XMLtag_ACCENT_prefix);
		l_xmlto_return.append("none");
		l_xmlto_return.append(XMLtag_ACCENT_suffix);
		l_xmlto_return.append(i_acc_word);
		l_xmlto_return.append(XMLtag_ACCENT_close);
				
		return l_xmlto_return.toString();

	}
	public static String HandleBoundaryTone(String i_bndry){
		if(debug){
			System.out.println("Handle boundary tones got: " + i_bndry);
		}
		String l_lhs =new String("");
		String l_rhs = new String("");;
		StringBuffer l_xmlto_return =new StringBuffer();
		l_xmlto_return.append(XMLtag_BOUNDARY_open);
		 l_xmlto_return.append(XMLtag_whitespace);
		 l_xmlto_return.append(XMLtag_TONE_prefix);
		
		 //RawMaryXML takes LL% as L-L%, !LH% as !L-H% and so on
		 if(i_bndry.length()>2 && i_bndry.length() <5 ){
			//e.g. LL%, H^H%,
			l_lhs=i_bndry.substring(0,1);
			l_rhs=i_bndry.substring(1);
		 }else if (i_bndry.length()<=2){
			//e.g. !H -> !H- , H -> L-
			l_lhs=i_bndry;
			l_rhs="";
		 }else if (i_bndry.length()==5){
			l_lhs=i_bndry.substring(0,2);
			l_rhs=i_bndry.substring(2);
		 }
				
		 l_xmlto_return.append(l_lhs);
		 l_xmlto_return.append(XMLtag_hyphen);
		 l_xmlto_return.append(l_rhs);
		 l_xmlto_return.append(XMLtag_TONE_suffix);
		 l_xmlto_return.append(XMLtag_whitespace);
		 l_xmlto_return.append(XMLtag_BREAKInd_prefix);
		 l_xmlto_return.append(XMLtag_BOUNDARY_pause);
		 l_xmlto_return.append(XMLtag_BREAKInd_suffix);
		 l_xmlto_return.append(XMLtag_BOUNDARY_close);
		  
		return l_xmlto_return.toString();
	}
	
	public static String WriteToXML(String i_xmlstring){
		if(debug){
			System.out.println("WriteToXML got: " + i_xmlstring);
			System.out.println("XML Head is: " + RAWMARYXMLHead);
		}
		
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
		StringBuffer l_xmlfilename = new StringBuffer();
		l_xmlfilename.append(UtteranceCount.toString());
		l_xmlfilename.append(stub);
		if (prosody) l_xmlfilename.append("_prsdy");
		else l_xmlfilename.append("_dflt");
		
		l_xmlfilename.append("_maryxml");
		
		Date l_date = new Date();
		Timestamp l_timestamp = new Timestamp(l_date.getTime());
		String l_datetime = new String(l_timestamp.toString());
		l_datetime=l_datetime.replace(':', '_');
		l_datetime=l_datetime.replace('.', '_');
		l_datetime=l_datetime.replace('-', '_');
		l_datetime=l_datetime.replace(' ', '_');
		
		l_xmlfilename.append(l_datetime);
		l_xmlfilename.append(".xml");
		
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
			BufferedWriter l_xml_out = new BufferedWriter(new FileWriter(GenratedXMLFileLocation.concat(l_xmlfilename.toString())));
			l_xml_out.write(l_rawxml_final.toString());
			l_xml_out.close();
			}
		catch (Exception fx) {
			System.out.println("Error in Write2XML: " + fx.toString());
		}
		return l_xmlfilename.toString();
	}
	//This is a function called from cc_TTS (changes in main() should also be reflected here)
	public static String ToRawMaryXml(String i_prsdyInp, String i_maryxmlheader, String i_outputLoc){
		//Save some global variables
		GenratedXMLFileLocation = i_outputLoc;
		RAWMARYXMLHead = i_maryxmlheader;
		stub="sys";
		prosody=true;
		
		return ConvertToRawMarxXml(i_prsdyInp);
	}
}
