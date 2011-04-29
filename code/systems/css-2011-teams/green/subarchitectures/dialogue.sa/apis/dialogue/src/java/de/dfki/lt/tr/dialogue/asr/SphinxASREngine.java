// =================================================================
// Copyright (C) 2010 DFKI GmbH Talking Robots 
// Geert-Jan M. Kruijff (gj@dfki.de)
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
package de.dfki.lt.tr.dialogue.asr;


//=================================================================
// IMPORT

// Java
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Properties;
import java.util.StringTokenizer;
import java.util.Vector;

import javax.xml.datatype.DatatypeConfigurationException;
import javax.sound.sampled.LineUnavailableException;

// Meta API
import de.dfki.lt.tr.meta.TRResultListener;

// Dialogue API slice
import de.dfki.lt.tr.dialogue.slice.asr.PhonString;

// Sphinx
import edu.cmu.sphinx.frontend.util.Microphone;
import edu.cmu.sphinx.recognizer.Recognizer;
import edu.cmu.sphinx.result.Lattice;
import edu.cmu.sphinx.result.LatticeOptimizer;
import edu.cmu.sphinx.result.Node;
import edu.cmu.sphinx.result.Result;
import edu.cmu.sphinx.util.props.ConfigurationManager;

/**
 * The <tt>SphinxASR</tt> class implements a basic engine around the
 * open-source Sphinx4 speech recognition API. 
 * 
 * <h4>Configuration options</h4>
 * 
 * The <tt>configure</tt> method takes a <tt>java.util.Properties</tt> 
 * object that can be used to configure the ASR engine. 
 * 
 * 
 * @see SphinxASR#configure(Properties)
 * @version 100618
 * @started 100618 
 */

public class SphinxASREngine {

	public final static String PHONSTRING = "phonString";
	public final static String WORDLATTICE = "wordLattice";
	public final static String LISTEN = "listen";
	public final static String SILENT = "silent";
	public final static String JSFG   = "JSFG";
	public final static String NGRAMS = "Ngrams";
	
	private boolean listenState; 
	private String	outputType;
	private String 	configFile; 
	
	private Vector<TRResultListener> listeners;
	
	private int idCounter;
	
	/**
	 * Constructor. Initializes the internal variables, but still requires a <tt>configuration</tt>.
	 * @see #configure(Properties)
	 */
	
	public SphinxASREngine () 
	{
		init();
	} // end constructor
	
	/**
	 * Constructs the engine and initializes it according to the provided configuration
	 * @param properties	The configuration properties
	 * @throws DatatypeConfigurationException	If there is a problem in configuring the engine
	 */
	
	public SphinxASREngine (Map<String,String> properties)
	throws DatatypeConfigurationException
	{
		init();
		this.configure(properties);
	} // end constructor
	
	/**
	 * Initializes the internal variables
	 */
	
	private void init () 
	{
		listenState = true;
		outputType  = PHONSTRING;
		configFile 	= null;
		idCounter  	= 0;
	} // end init
	
	/**
	 * Configures the Sphinx4 speech recognition engine. 
	 * 
	 * <ul>
	 * <li> --listenOnInit: Whether to start listening from the start. Listening state can be altered
	 * 		using the <tt>turnOnListening/turnOffListening</tt> methods. Value either SphinxASR.LISTEN 
	 * 		or SphinxASR.SILENT; by default listening is on from the start. 
	 * <li> --outputType: What kind of output recognition result to produce. Value either 
	 * 		SphinxASR.PHONSTRING or SphinxASR.WORDLATTICE; default is PHONSTRING. 
	 * <li> --configFile: value is a filename of a Sphinx configuration file
	 * </ul>
	 * 
	 * @param properties
	 * @throws DatatypeConfigurationException Thrown if there is a problem configuring the engine
	 */
	
	public void configure (Map<String,String> properties)
	throws DatatypeConfigurationException
	{
		if (properties == null) 
		{
			throw new DatatypeConfigurationException("Cannot configure SphinxASR: null configuration");
		}
		String listenOnInit = properties.get("--listenOnInit");
		if (listenOnInit == null || listenOnInit.equals(LISTEN)) 
		{
			this.listenState = true;
		} else {
			this.listenState = false;
		} // end if.. check 
		String outputType = properties.get("--outputType");
		if (outputType != null && outputType.equals(WORDLATTICE))
		{
			this.outputType = WORDLATTICE;
		} else {
			this.outputType = PHONSTRING;
		}
		// check whether we have a grammar
		configFile = properties.get("--configFile");
		if (configFile == null) 
		{
			throw new DatatypeConfigurationException 
					("Cannot configure SphinxASR: missing configuration file / property [configFile]"); 
		} // end if..
	} // end configure
	
	/**
	 * Turns on listening to input on the microphone
	 */
	
	public void turnOnListening ()
	{
		listenState = true;
	} // end turnOnListening
	
	/**
	 * Turns off listening to input on the microphone 
	 */
	
	public void turnOffListening ()
	{
		listenState = false;
	}
	
	/**
	 * Registers a process with the engine, to be notified whenever
	 * the engine produces a new result
	 * @param listener		The process to be informed
	 * @throws UnsupportedOperationException Thrown if the process is null
	 */
	
	public void registerNotification (TRResultListener listener)
	throws UnsupportedOperationException
	{
		if (listener == null) 
		{
			throw new UnsupportedOperationException("Cannot register process to be notified: "
					+"Provided process is null");	
		} else {
			this.listeners.add(listener); 
		}		
	} // end registerNotification
	
	/**
	 * Notify all the registered listeners of the obtained result. If none have been registered, 
	 * the result is dumped onto System.out.
	 * @param result
	 */
	
	private void notify (Object result)
	{
		if (listeners.isEmpty()) {
			PhonString phonStr = (PhonString) result;
			System.out.println("Recognized: "+phonStr.wordSequence);
		} else {
			// Notify all the registered listeners
			for (TRResultListener listener : listeners)
			{
				listener.notify(result);
			} // end for 
		}
	} // end notify
	
	private String newId() 
	{ 
		idCounter++;
		return "asr"+idCounter;
	}
	
	/**
	 * Starts the engine. The engine is run in an infinite loop. Listening can be turned on and off
	 * using the appropriate methods. 
	 * 
	 *  @see #turnOnListening()
	 *  @see #turnOffListening()
	 *  @throws DatatypeConfigurationException Thrown if there is no Sphinx4 configuration file 
	 *  @throws LineUnavailableException Thrown if the microphone cannot be opened
	 */
	
	public void run ()
	throws DatatypeConfigurationException, LineUnavailableException
	{
		if (configFile == null)
		{
			throw new DatatypeConfigurationException ("Cannot run SphinxASR: missing configuration file"); 
		}
		// Load the configuration
        ConfigurationManager cm = new ConfigurationManager(configFile);
        // Initialize the recognizer
        Recognizer recognizer = (Recognizer) cm.lookup("recognizer");
        recognizer.allocate();
        // Start the microphone or throw an exception if this is not possible
        Microphone microphone = (Microphone) cm.lookup("microphone");
        if (!microphone.startRecording()) {
            recognizer.deallocate();
            throw new LineUnavailableException ("Cannot run SphinxASR: microphone unavailable");
        }
        // loop infinitely; listening can be dynamically be turned on or off
        while (true) {
        	if (listenState) {
        		Result result = recognizer.recognize();
        		if (result != null) {
        			if (outputType.equals(PHONSTRING)) 
        			{
            			String resultText = result.getBestFinalResultNoFiller();
            			StringTokenizer tokenizer = new StringTokenizer(resultText);
            			PhonString phonString = new PhonString();
            			phonString.id = newId();
            			phonString.wordSequence= resultText;
            			phonString.length= tokenizer.countTokens();
            			phonString.confidenceValue = 1.0f;
            			phonString.NLconfidenceValue= 0.0f;
            			phonString.rank = 1;  
            			notify((Object)phonString);
        			} else if (outputType.equals(WORDLATTICE))
        			{
                        Lattice lattice = new Lattice(result);
                        LatticeOptimizer optimizer = new LatticeOptimizer(lattice);
                        optimizer.optimize();
                        // next, eithe construct the viterbi path, or store the entire lattice
        				List<Node> viterbiPath = lattice.getViterbiPath();
        				// not fully yet supported
        			}        		
        		} else {
        			// provide feedback
        		} // end if..else check for result
        	} // end if.. check to listen
        } // end while       
	} // end run
	
	
	/**
	 * Main for just running the sphinx engine in test mode.
	 * Start with a single argument: <tt>java de.dfki.lt.tr.dialogue.asr.SphinxASREngine configfile</tt>
	 */
	
	public static void main (String[] args) 
	{
		try {
			SphinxASREngine theLittleEngineThatCould = new SphinxASREngine();
			HashMap<String,String> configs = new HashMap<String,String>();
			configs.put("--configFile", args[0]);
			theLittleEngineThatCould.configure(configs);
			theLittleEngineThatCould.run();
		} catch (LineUnavailableException lue) {
			lue.printStackTrace();
			System.exit(0);
		} catch (DatatypeConfigurationException dce) {
			dce.printStackTrace();
			System.exit(0);
		} // end try..catch
		
	} // end main
	
	
} // end class
