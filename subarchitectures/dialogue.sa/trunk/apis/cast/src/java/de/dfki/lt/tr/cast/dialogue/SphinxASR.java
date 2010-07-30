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
package de.dfki.lt.tr.cast.dialogue;

//=================================================================
// IMPORT

//Java
import java.util.Map;
import javax.sound.sampled.LineUnavailableException;
import javax.xml.datatype.DatatypeConfigurationException;

//CAST
import cast.core.CASTData;


// Dialogue API CAST
import de.dfki.lt.tr.cast.ProcessingData;

// Dialogue API slice
import de.dfki.lt.tr.dialogue.slice.asr.PhonString;

// Dialogue API 
import de.dfki.lt.tr.dialogue.asr.SphinxASREngine;
import de.dfki.lt.tr.dialogue.util.DialogueException;

// Meta API
import de.dfki.lt.tr.meta.TRResultListener;

/**
 * <tt>SphinxASR</tt> provides a speech recognition component, based 
 * on the open-source Sphinx4 ASR engine. Whenever the engine produces a <tt>PhonString</tt> object, 
 * the component proposes a speech.rec task. If this task is accepted, the </tt>PhonString</tt> is 
 * written out to working memory by the <tt>executeTask</tt> method. 
 * 
 * <h4>CAST component line configuration</h4>
 * The configuration options are those as described for the Sphinx ASR engine. 
 * @see de.dfki.lt.tr.dialogue.asr.SphinxASREngine#configure(Map)
 * 
 * @author Geert-Jan M. Kruijff
 * @version 100622
 * @start   100622
 */


public class SphinxASR 
extends AbstractDialogueComponent
implements TRResultListener
{
	
	SphinxASREngine sphinxEngine; 
	

	/**
	 * Starts up the component. The engine is configured and started 
	 * already in the configure method (which is called before start). 
	 * 
	 * @see #configure(Map)
	 */
	
	@Override
    public void start() {
        super.start();
    } // end start
	
    /**
     * Uses the component configuration arguments to configure the ASR engine, 
     * and starts the engine (<tt>run</tt>) once it has been configured. 
     */
    
	@Override
    public void configure (Map<String, String> _config) 
    {
    	try { 
            sphinxEngine = new SphinxASREngine();
            sphinxEngine.registerNotification(this);
        	sphinxEngine.configure(_config);
        	sphinxEngine.run();
    	} catch (LineUnavailableException lue)	{
    		lue.printStackTrace();
    		System.exit(0);
    	} catch (DatatypeConfigurationException dce) {
    		dce.printStackTrace();
    		System.exit(0);
    	} // end try..catch
    } // end 
    
	
	/**
	 * Called whenever the ASR engine has a PhonString available. 
	 * The method pushes the resulting PhonString as a Task. 
	 * 
	 * @param result The result from the ASR engine
	 */
		
	@Override
	public void notify (Object result)
	{
		// create a CAST data object for the result
        CASTData data = new CASTData ("emptyid",(PhonString)result);
        // get a new id for the task
        String taskID = newTaskID();
        // store the data we want to process for later
        ProcessingData pd = new ProcessingData(
            newProcessingDataId());
        pd.add(data);
        m_proposedProcessing.put(taskID, pd);
		// set the goal
        String taskGoal = DialogueGoals.ASR_TASK;
        System.out.println("ID: " + taskID);
        System.out.println("goal: " + taskGoal);
		// store the goal with its information
        proposeInformationProcessingTask(taskID, taskGoal);
	}
	
	/**
	 * Implements the execution task: If the task ASR_TASK is 
	 * accepted, the (already) constructed PhonString is passed
	 * on to working memory. 
	 * 
	 * @param data	The data structure containing the PhonString object
	 */
	
	@Override
	public void executeTask (ProcessingData data)
	throws DialogueException
	{
    	try {
    		PhonString phonString = (PhonString) data.getData();
    		addToWorkingMemory(newDataID(),phonString); 
    	}
    	catch (Exception e) {
    		e.printStackTrace();
    		throw new DialogueException(e.getMessage());
    	} // end try..catch overwriting working memory
	} // end executeTask
		
} // end class
