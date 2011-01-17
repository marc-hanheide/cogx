package comsys.components.tts;

import java.io.BufferedReader;
import java.io.FileReader;
//import marytts.client.MaryClient;
import de.dfki.lt.mary.client.*;

import comsys.components.tts.TTSLocal;
//import marytts.client.http.Address;

public class SynthesisRAWMaryXMLInput {
		public  TTSLocal m_ttsLocal;
	
	
public SynthesisRAWMaryXMLInput(TTSLocal i_ttslocal){
	this.m_ttsLocal=i_ttslocal;
}
	/**
	 * @param args
	 */
	public static void main(String[] args){
		try {
			MaryClient l_mary = new MaryClient("localhost", 59125);
			TTSLocal l_ttslocal = new TTSLocal(l_mary,"RAWMARYXML", args[1], false, "WAVE");
			l_ttslocal.m_AudioFileName=args[1];
			l_ttslocal.m_SaveAudio2Wav=Boolean.valueOf(args[2]);
			SynthesisRAWMaryXMLInput l_synth = new SynthesisRAWMaryXMLInput(l_ttslocal);
			l_synth.Utter(args[1]);
		}
        catch (Exception e) {
        	System.out.println(e);
        }
     }
	
	public void Utter(String i_filename) {
		
       //System.out.println("Synthesize RAWMARYXML file: "+ i_filename);
       	StringBuffer output = new StringBuffer();
		try {
			BufferedReader in = new BufferedReader(new FileReader(i_filename));
			String st;
			while ((st=in.readLine()) != null) {
			output.append(st);
			output.append(" ");
			}
			in.close();
		}
		catch (Exception fx) {
			System.out.println("IO error in Synthesis: " + fx.toString());
		}
		m_ttsLocal.speak(output.toString());
	}
	
	public void Save2Wave(String i_filename) {
		
	       System.out.println("Synthesize RAWMARYXML file: "+ i_filename);
	       	StringBuffer output = new StringBuffer();
			try {
				BufferedReader in = new BufferedReader(new FileReader(i_filename));
				String st;
				while ((st=in.readLine()) != null) {
				output.append(st);
				output.append(" ");
				}
				in.close();
			}
			catch (Exception fx) {
				fx.printStackTrace();
				System.out.println("IO error in Synthesis: " + fx.toString());
			}
			m_ttsLocal.SaveToFile(output.toString());
		}

}
