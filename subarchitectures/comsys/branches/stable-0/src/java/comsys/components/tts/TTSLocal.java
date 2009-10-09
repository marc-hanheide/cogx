
//  TTSLocal.java
//
//  Created by Pierre Lison on 13/03/07.
//  Copyright 2007 DFKI/Cosy . All rights reserved.
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
// ===================//

package comsys.components.tts;

import java.io.ByteArrayInputStream;
import java.io.ByteArrayOutputStream;

import javax.sound.sampled.*;
import de.dfki.lt.mary.client.MaryClient;

import de.dfki.lt.signalproc.util.AudioPlayer;


import java.io.File;
/**
 * TTS system for local synthesis (ie. using directly accessible speakers).
 */
public class TTSLocal {

	// mary client
	private  MaryClient m_mary ;
	
   	// basic TTS parameters
	private  String m_inputType = "TEXT_EN";
	//private  String m_inputType = "RAWMARYXML";
	private  String m_outputType = "AUDIO";
	private  String m_audioType = "WAVE";
	//private  String m_locale = "en_US"; for Mary.4
	
	// The name of the voice to be used
	//private   String m_voiceName = "female";
	private   String m_voiceName = "us2"; //MBROLO sounds
		
	// System lock
	protected boolean m_isFree = true ;
	
	// Silence
	private boolean m_bSilentMode;
	
   
	/** 
	 * Create a local TTS system.
	 *
	 * @param mary A mary client
	 * @param voiceName a string representing the voice name 
	          (must correspond to one voice installed on the mary server)
	* @param m_bSilentMode true if system must remain silent, false otherwise
	*/
	
	public TTSLocal (MaryClient mary, String inputType, String voiceName, boolean bSilentMode,  String audioType) {
			this.m_mary = mary ;
			this.m_inputType = inputType;
			this.m_voiceName = voiceName ;
			this.m_bSilentMode = bSilentMode ;
			this.m_audioType = audioType;
			
	}
	/* for Mary.4
	public TTSLocal (MaryClient mary, String voiceName, boolean bSilentMode, String locale, String audioType) {
		this.m_mary = mary ;
		this.m_voiceName = voiceName ;
		this.m_bSilentMode = bSilentMode ;
		this.m_audioType = audioType;
		this.m_locale=locale;
}*/

	
	/**
	 * Utters something using the local TTS system.
	 *
	 * <p> Note: at least one audio line (ie. speakers) must be available and accessible 
	 * on the local machine.
	 * 
	 * @param tosay the string to utter
	 */
	   public void  speak(String tosay)   {
		   if (m_bSilentMode)
			   System.out.println("(silent mode) - \"" + tosay + "\"");
		   
		   else {
		   try {
		        ByteArrayOutputStream baos = new ByteArrayOutputStream();
		        //m_mary.process(tosay, m_inputType, m_outputType, m_locale, m_audioType, m_voiceName, baos); for Mary.4
		        m_mary.process(tosay, m_inputType, m_outputType, m_audioType, m_voiceName, baos);
		        
		        
			   AudioInputStream ais = AudioSystem.getAudioInputStream(
			            new ByteArrayInputStream(baos.toByteArray()));
			        LineListener lineListener = new LineListener() {
			            public void update(LineEvent event) {
			                if (event.getType() == LineEvent.Type.START) {
			                    System.err.println("Audio started playing.");
			                } else if (event.getType() == LineEvent.Type.STOP) {
			                    System.err.println("Audio stopped playing.");
			                } else if (event.getType() == LineEvent.Type.OPEN) {
			                    System.err.println("Audio line opened.");
			                } else if (event.getType() == LineEvent.Type.CLOSE) {
			                    System.err.println("Audio line closed.");
			                }
			            }
			        };

			        AudioPlayer ap = new AudioPlayer(ais, lineListener);
			        ap.start();
			   
		   }
		   catch (Exception e) {e.printStackTrace() ; } 
		   }
		   
	   } // end speak
	
		/**
		 * Utters something using the local TTS system.
		 *
		 * <p> Note: at least one audio line (ie. speakers) must be available and accessible 
		 * on the local machine.
		 * 
		 * @param tosay the string to utter
		 */
		   public void  saveToFile(String tosay, String filename)   {
			   if (m_bSilentMode)
				   System.out.println("(silent mode) - \"" + tosay + "\"");
			   
			   else {
			   try {
			        ByteArrayOutputStream baos = new ByteArrayOutputStream();
			         m_mary.process(tosay, m_inputType, m_outputType,  m_audioType, m_voiceName, baos);
			         //m_mary.process(tosay, m_inputType, m_outputType, m_locale, m_audioType, m_voiceName, baos);  for Mary.4
			        File file = new File(filename);
				   AudioInputStream ais = AudioSystem.getAudioInputStream(
				            new ByteArrayInputStream(baos.toByteArray()));
				       AudioSystem.write(ais, AudioFileFormat.Type.WAVE, file);
				   
			   }
			   catch (Exception e) {e.printStackTrace() ; } 
			   }
			   
		   } // end speak
		
			
}
