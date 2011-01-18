
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
import marytts.client.MaryClient;
import marytts.util.data.audio.AudioPlayer;


import java.io.File;
import java.sql.Timestamp;
import java.util.Date;
/**
 * TTS system for local synthesis (ie. using directly accessible speakers).
 */
public class TTSLocal {

	// mary client
	private  MaryClient m_mary ;

	// basic TTS parameters
	public  String m_inputType = "TEXT_EN";
	private  String m_outputType = "AUDIO";
	private  String m_audioType = "WAVE";
	private  String m_locale = "en_US";// Default NLP Unit
	private String m_effect= "";
	
	// The name of the voice to be used
	public String m_voiceName = "us2"; //Default male sounds
		
	// System lock
	protected boolean m_isFree = true ;
	
	// Silence
	private boolean m_bSilentMode;
	
	//Prosodic changes 
	private ByteArrayOutputStream m_baos;
	
	//Save wav file
	public  boolean m_SaveAudio2Wav=false;
	public  String m_AudioFileName=null;
	
	/** 
	 * Constructor for a local TTS
	 * @param mary  an Instance of Mary client
	 * @param inputType Type of input: text, rawmaryxml,...
	 * @param voiceName us2, de7,...Mary voices for speech synthesis 
	 * (must correspond to one voice installed on the Mary server)
	 * @param bSilentMode true if system must remain silent, false otherwise
	 * @param audioType	Speech output format e.g. WAV, AUDIO
	*/
	public TTSLocal (MaryClient mary, String inputType, String voiceName, boolean bSilentMode,  String audioType) {
			this.m_mary = mary ;
			this.m_inputType = inputType;
			this.m_voiceName = voiceName ;
			this.m_bSilentMode = bSilentMode;
			this.m_audioType = audioType;
							
	}
	
	/**
	 * This constructor takes the Effect type argument in addition to the other standard parameters.
	 * @param mary  an Instance of Mary client
	 * @param inputType Type of input: text, rawmaryxml,...
	 * @param voiceName us2, de7...Mary voices for speech synthesis
	 * @param bSilentMode silent (no idea what this is)
	 * @param audioType	Speech output format e.g. WAV, AUDIO
	 * @param effectType Effect. Ref. MaryClient GUI.
	 * @param locale a parameter used to distinguish language specific NLP modules in MARY.
	 * e.g. 'de', 'en_US', 'en_GB'
	 */
	public TTSLocal (MaryClient mary, String inputType, String voiceName, boolean bSilentMode,  String audioType, String effectType, String nlpUnit) {
		this(mary,inputType,voiceName,bSilentMode,audioType);
		this.m_effect = effectType;
		this.m_locale = nlpUnit;
	}

	
	/**
	 * Utters something using the local TTS system.
	 *
	 * <p> Note: at least one audio line (ie. speakers) must be available and accessible 
	 * on the local machine.
	 * 
	 * @param tosay the string to utter
	 */
	   public void  speak(String tosay)   {
		   if (m_bSilentMode)   System.out.println("(silent mode) - \"" + tosay + "\"");
		   else {
			   try {
			   		m_baos = new ByteArrayOutputStream();
			   		m_mary.process(tosay, m_inputType, m_outputType, m_locale, m_audioType, m_voiceName, "", m_effect, null, m_baos);
			    
			   		AudioInputStream ais = AudioSystem.getAudioInputStream(
			   								new ByteArrayInputStream(m_baos.toByteArray()));
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
			        
			        
			        if(m_SaveAudio2Wav){
			        	 
			        	 //File name
			        	m_AudioFileName=m_AudioFileName.replaceAll(".xml",".wav");
			        	System.out.println("Wave file saved to: "+ m_AudioFileName );
			        	File file = new File(m_AudioFileName);
			        	AudioInputStream ais_w = AudioSystem.getAudioInputStream(
							     new ByteArrayInputStream(m_baos.toByteArray()));
			        	AudioSystem.write(ais_w, AudioFileFormat.Type.WAVE, file);
					  }
			       			   
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
		   public void  SaveToFile(String tosay)   {
			   if (m_bSilentMode)
				   System.out.println("(silent mode) - \"" + tosay + "\"");
			   
			   else {
			   try {
			         
					m_baos = new ByteArrayOutputStream();
			       	m_mary.process(tosay, m_inputType, m_outputType, m_locale, m_audioType, m_voiceName, "", m_effect, null, m_baos);		   	 
				   	m_AudioFileName=m_AudioFileName.replaceAll(".xml",".wav");
		        	System.out.println("Wave file saved to: "+ m_AudioFileName );
		        	
				   	 File file = new File(m_AudioFileName);
				     AudioInputStream ais = AudioSystem.getAudioInputStream(
				     new ByteArrayInputStream(m_baos.toByteArray()));
				     AudioSystem.write(ais, AudioFileFormat.Type.WAVE, file);
				   
			   }
			   catch (Exception e) {e.printStackTrace() ; } 
			   }
			   
		   } // end SpeakAndSaveToFile
		
			
}
