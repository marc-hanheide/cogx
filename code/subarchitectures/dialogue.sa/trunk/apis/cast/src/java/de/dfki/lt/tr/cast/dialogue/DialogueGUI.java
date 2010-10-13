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


// Java
import java.awt.event.WindowAdapter;
import java.awt.event.WindowEvent;
import java.util.Map;

// CAST
import cast.SubarchitectureComponentException;
import cast.architecture.ChangeFilterFactory;
import cast.architecture.WorkingMemoryChangeReceiver;
import cast.cdl.WorkingMemoryChange;
import cast.cdl.WorkingMemoryOperation;
import cast.core.CASTData;

// Dialogue API CAST
import de.dfki.lt.tr.cast.ProcessingData;

//Dialogue API slice
import de.dfki.lt.tr.dialogue.slice.asr.PhonString;
import de.dfki.lt.tr.dialogue.slice.synthesize.SpokenOutputItem;

//Dialogue API 
import de.dfki.lt.tr.dialogue.util.DialogueChatWindow;
import de.dfki.lt.tr.dialogue.util.DialogueException;

// META
import de.dfki.lt.tr.meta.TRResultListener;
import java.util.Iterator;

// =================================================================


/**
 * The <tt>DialogueGUI</tt> component displays a simple chat window, 
 * in which the user can type in an utterance. The window displays the 
 * entire dialogue, in terms of what utterances the user has typed, 
 * and what responses the robot (might have) generated. Any utterance
 * typed in here gets posted as a <tt>PhonString</tt> object to a
 * Working Memory. The component listens for <tt>SpokenOutputItem</tt> 
 * data on a WM, (which are the robot responses). 
 * 
 * @author 	Geert-Jan M. Kruijff (gj@dfki.de)
 * @version 100623
 * @started 100623
 */

public class DialogueGUI 
extends AbstractDialogueComponent
implements TRResultListener
{
	
	DialogueChatWindow gui;
	
	/**
	 * Starts up the component. The GUI is configured and started 
	 * already in the configure method (which is called before start). 
	 * 
	 * @see #configure(Map)
	 */
	
    @Override
    public void start() {
        super.start();
		// register change filters for PhonString, which triggers parsing
        addChangeFilter(
        		ChangeFilterFactory.createLocalTypeFilter(SpokenOutputItem.class,  WorkingMemoryOperation.ADD),
            new WorkingMemoryChangeReceiver() {
                public void workingMemoryChanged(WorkingMemoryChange _wmc) {
                	handleWorkingMemoryChange(_wmc);
                }
            });
    } // end start
	
    /**
     * Whenever a SpokenOutputItem is picked up, it's provided to the chat window for display. 
     * @param _wmc
     */
    
    private void handleWorkingMemoryChange (WorkingMemoryChange _wmc)
    {
        try {
            // get the data from working memory and publish it
            gui.publishSOI(getMemoryEntry(_wmc.address, SpokenOutputItem.class));
        }
        catch (SubarchitectureComponentException e) {
            e.printStackTrace();
        } // end try..catch
    }
    
    /**	
     *  Initializes and displays the GUI
     */
   
    @Override
    public void configure (Map<String, String> _config) 
    {
    	gui = new DialogueChatWindow();
    	gui.addWindowListener(new WindowAdapter() 
		{
                        @Override
			public void windowClosing(WindowEvent e) 
			{
				System.exit(0);
			}
		}
		); // end addWindowListener
    	gui.pack();
    	gui.setVisible(true);
    	gui.registerNotification(this);
    } // end configure
    
    
	/**
	 * Called whenever the Dialogue chat window makes a PhonString available. 
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
//        System.out.println("ID: " + taskID);
//        System.out.println("goal: " + taskGoal);
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
                Iterator<CASTData> it = data.getData();
                while (it.hasNext()) {
                    PhonString phonString = (PhonString) it.next().getData();
                    if (phonString.wordSequence.equals("no")) {
                    	phonString.wordSequence = "No";
                    }
                    addToWorkingMemory(newDataID(), phonString);
                }
    	}
    	catch (Exception e) {
    		e.printStackTrace();
    		throw new DialogueException(e.getMessage());
    	} // end try..catch overwriting working memory
	} // end executeTask
	
	
	
} // end class
