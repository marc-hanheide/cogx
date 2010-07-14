// ====================================================================//
//  MaryTTSLocal.java
//
//  Created by Pierre Lison on 13/03/07.
//  Copyright 2007 DFKI GmbH . All rights reserved.
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
// ====================================================================//

package de.dfki.lt.tr.dialogue.tts;

import java.io.ByteArrayInputStream;
import java.io.ByteArrayOutputStream;

import javax.sound.sampled.*;

import de.dfki.lt.mary.client.MaryClient;

import de.dfki.lt.signalproc.util.AudioPlayer;
import de.dfki.lt.mary.client.MaryClient.AudioPlayerListener;
import java.io.File;

/**
 * TTS system for local synthesis (ie. using directly accessible speakers).
 */
public class MaryTTSLocal {


    // The name of the voice to be used
   String voiceName = "female";
	
	// mary client
	MaryClient mary ;
	
	// basic TTS parameters
	static String inputType = "TEXT_EN";
	static String outputType = "AUDIO";
	static String audioType = "WAVE";
	
	// System lock
	protected boolean isFree = true ;
	
	// Silence
   boolean m_bSilentMode;
	
   
	/** 
	 * Create a local TTS system.
	 *
	 * @param mary A mary client
	 * @param voiceName a string representing the voice name 
	          (must correspond to one voice installed on the mary server)
	* @param m_bSilentMode true if system must remain silent, false otherwise
	*/
	
	public MaryTTSLocal (MaryClient mary, String voiceName, boolean m_bSilentMode, String audioType) {
			this.mary = mary ;
			this.voiceName = voiceName ;
			this.m_bSilentMode = m_bSilentMode ;
			this.audioType = audioType;
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
		   if (m_bSilentMode)
			   System.out.println("(silent mode) - \"" + tosay + "\"");
		   
		   else {
		   try {
		        ByteArrayOutputStream baos = new ByteArrayOutputStream();
		        mary.process(tosay, inputType, outputType, audioType,
		        		voiceName, baos);
		        
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
			        mary.process(tosay, inputType, outputType, audioType,
			        		voiceName, baos);
			        
			        File file = new File(filename);
				   AudioInputStream ais = AudioSystem.getAudioInputStream(
				            new ByteArrayInputStream(baos.toByteArray()));
				       AudioSystem.write(ais, AudioFileFormat.Type.WAVE, file);
				   
			   }
			   catch (Exception e) {e.printStackTrace() ; } 
			   }
			   
		   } // end speak
		
			
}
