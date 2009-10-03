package comsys.components.tts;

import java.io.BufferedReader;
import java.io.FileReader;
//import marytts.client.MaryClient;
import de.dfki.lt.mary.client.*;

import comsys.components.tts.TTSLocal;
//import marytts.client.http.Address;

public class SynthesisRAWMaryXMLInput {
	public static MaryClient m_mary;
	public static TTSLocal m_ttsLocal;

	/**
	 * @param args
	 */
	public static void Utter(String i_filename) {
		// TODO Auto-generated method stub
		
		try {
			//m_mary = MaryClient.getMaryClient(new Address("localhost", 59125));
			m_mary = new MaryClient("localhost", 59125);
			}
        catch (Exception e) {
        	System.out.println(e);
        }
         
       //	m_ttsLocal = new TTSLocal(m_mary, "female", false, "en_US", "WAVE");
    	m_ttsLocal = new TTSLocal(m_mary,"RAWMARYXML", "us2", false, "WAVE");
		
       	System.out.println("input filename: "+ i_filename);
       	
       	StringBuffer output = new StringBuffer();
		try {
			BufferedReader in = new BufferedReader(new FileReader(i_filename));
			
			String st;
			while ((st=in.readLine()) != null) {
			output.append(st);
			output.append(" ");
			
			}
			System.out.println(output.toString());
			in.close();
			}
		catch (Exception fx) {
			System.out.println("IO error in Synthesis : " + fx.toString());
		}
		
		m_ttsLocal.speak(output.toString());
	}

}
