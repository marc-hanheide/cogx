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

import java.io.BufferedReader;
import java.io.FileReader;
import java.util.concurrent.CountDownLatch;

import marytts.client.MaryClient;
//import marytts.client.MaryClient;

import de.dfki.lt.tr.dialogue.tts.MaryTTSLocal;
//import marytts.client.http.Address;

public class SynthesisRAWMaryXMLInput {
		public  MaryTTSLocal m_ttsLocal;
	
	
public SynthesisRAWMaryXMLInput(MaryTTSLocal i_ttslocal){
	this.m_ttsLocal=i_ttslocal;
}
	/**
	 * @param args
	 */
	public static void main(String[] args){
		try {
			MaryClient l_mary = MaryClient.getMaryClient();
			MaryTTSLocal l_ttslocal = new MaryTTSLocal(l_mary,"RAWMARYXML", args[1], false, "WAVE");
			l_ttslocal.m_AudioFileName=args[1];
			l_ttslocal.m_SaveAudio2Wav=Boolean.valueOf(args[2]);
			SynthesisRAWMaryXMLInput l_synth = new SynthesisRAWMaryXMLInput(l_ttslocal);
			l_synth.Utter(args[1], new CountDownLatch(1));
		}
        catch (Exception e) {
        	System.out.println(e);
        }
     }
	
	public void Utter(String i_filename, CountDownLatch finished) {
		
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
		m_ttsLocal.speak(output.toString(), finished);
	}
	
	public void Save2Wave(String i_filename) {
		
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
			m_ttsLocal.SaveToFile(output.toString());
		}

}
