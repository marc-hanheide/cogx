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
//IMPORT

//Java
import cast.SubarchitectureComponentException;
import java.io.File;
import java.util.Map;

import javax.sound.sampled.LineUnavailableException;
import javax.xml.datatype.DatatypeConfigurationException;

//CAST
import cast.architecture.ChangeFilterFactory;
import cast.architecture.WorkingMemoryChangeReceiver;
import cast.cdl.WorkingMemoryChange;
import cast.cdl.WorkingMemoryOperation;
import cast.core.CASTData;

// Dialogue API Slice
import de.dfki.lt.tr.dialogue.slice.produce.ContentPlanningGoal;
import de.dfki.lt.tr.dialogue.slice.produce.ProductionLF;
import de.dfki.lt.tr.dialogue.slice.lf.*;

//Dialogue API CAST
import de.dfki.lt.tr.cast.ProcessingData;
import de.dfki.lt.tr.cast.dialogue.DialogueGoals;

// Dialogue API
import de.dfki.lt.tr.dialogue.asr.SphinxASREngine;
import de.dfki.lt.tr.dialogue.util.DialogueException;
import de.dfki.lt.tr.dialogue.util.LFUtils;
import de.dfki.lt.tr.dialogue.cplanwrapper.CPlanWrapper;
import java.util.Iterator;

/**
 * Provides a CAST component for the content planner. The content planner takes as input a (proto) logical form,
 * provided by dialogue management as a ContentPlanningGoal object on WM, and then plans a complete logical form for it.
 * The resulting logical form is written out to working memory as a ProductionLF object.
 *
 * <h4>CAST component line configuration</h4>
 * <ul>
 * <li>  <tt>--domainFile [fileName]</tt> specifies the top-file for the content planning domain to be loaded. </li>
 * <li>  <tt>--contentRel [String] </tt> specifies whether the content is, under the root of a planned logical form.
 * 										If none is specified, the entire planned LF is written out as ProductionLF. </li>
 * </ul>
 *
 * @author Geert-Jan M. Kruijff
 * @version 100709
 * @started 100707
 */


public class ContentPlanner
extends AbstractDialogueComponent
{
	/* Relation for the canned text */
	private String cannedText  = "";

	private CPlanWrapper planner = null;
	private	String contentRel = null;


	/**
	 * Starts up the component. The planner is configured and started
	 * already in the configure method (which is called before start).
	 * The start method registers a listener for ContentPlanningGoal objects on the WM
	 *
	 * @see #configure(Map)
	 */

	@Override
    public void start() {
        super.start();
		addChangeFilter(
				ChangeFilterFactory.createLocalTypeFilter(ContentPlanningGoal.class,  WorkingMemoryOperation.ADD),
				new WorkingMemoryChangeReceiver() {
					@Override
					public void workingMemoryChanged(WorkingMemoryChange _wmc) {
						handleWorkingMemoryChange(_wmc, DialogueGoals.CONTENTPLANNING_TASK);
					}
				});
    } // end start


	private void handleWorkingMemoryChange(WorkingMemoryChange _wmc, String taskGoal) {
		try {
			CASTData data = getWorkingMemoryEntry(_wmc.address.id);
			ContentPlanningGoal cpg = (ContentPlanningGoal)data.getData();

			ProcessingData pd = new ProcessingData(newProcessingDataId());
			pd.add(data);
			String taskID = newTaskID();
			m_proposedProcessing.put(taskID, pd);
//			System.out.println("ID: " + taskID);
//			System.out.println("goal: " + taskGoal);
			proposeInformationProcessingTask(taskID, taskGoal);
		}
		catch (SubarchitectureComponentException e) {
			e.printStackTrace();
		}
	}

    /**
     * Configuration of the content planner
     * Available flags:
     * <ul>
     * <li> <tt>--domainFile</tt> pointer to the top file to be loaded
     * <li> <tt>--contentRel</tt> name of the content relation (if any) in a production LF
     * </ul>
     */

	@Override
    public void configure (Map<String, String> _config)
    {
    	if (_config.containsKey("--domainFile"))
    	{
    	  // TODO the second argument should be the path to the directory where
    	  // the function plugin jar(s) can be found
    		File domainFile = new File(_config.get("--domainFile"));
   			planner = new CPlanWrapper(domainFile, null);
    	}
    	if (_config.containsKey("--contentRel"))
    	{
    		contentRel = _config.get("--contentRel");
    	}
    	if (_config.containsKey("--cannedText"))
    	{
    		cannedText = _config.get("--cannedText");
    	} else {
    		cannedText = "CannedText";
    	}
    
    } // end

    /**
     * Given a content planning goal, the content planner is called to produce a full logical form realizing that goal.
     * If a <tt>contentRel</tt> was specified in the component configuration, then the logical form under that relation under the root
     * is written as ProductionLF to working memory. Otherwise, the complete logical form as provided by the planner is stored on
     * working memory.
     *
     */

	@Override
	public void executeTask (ProcessingData data)
	throws DialogueException
	{
    	try {
			Iterator<CASTData> iter = data.getData();
			while (iter.hasNext()) {
	    		// Get the proto structure from the data
				CASTData pdata = iter.next();
				ContentPlanningGoal goal = (ContentPlanningGoal) pdata.getData();
				String canned = LFUtils.lfNominalGetFeature(goal.lform.root, cannedText);
				log("Canned text feature value ["+goal.lform.root.nomVar+"]: "+canned);
				// Put the goal to the content planner
				if (planner != null && canned.equals(""))
				{
					LogicalForm output = planner.callContentPlanner(goal.lform);				
					ProductionLF productionLF = new ProductionLF();
					if (contentRel != null)
					{
						try {
							// retrieve the subtree from under the contentRel relation under the root of the LF
							LFRelation crel = LFUtils.lfNominalGetRelation(output.root,contentRel);
							LFNominal croot = LFUtils.lfGetNominal(output,crel.dep);
							LogicalForm content = LFUtils.lfConstructSubtree(croot,output);
							// store it in the production LF
							productionLF.lform = content;
						} catch (NullPointerException npe) {
							System.out.println(npe.getStackTrace());
						}
					} else {
						productionLF.lform = output;
					}
					productionLF.plfid = newId("cp");
					productionLF.topic = goal.topic;
					// Put the resulting logical form onto WM
					log("adding the production LF to the WM: " + LFUtils.lfToString(productionLF.lform));
					addToWorkingMemory(newDataID(),productionLF);
				} else {
					// Put the resulting logical form onto WM
					ProductionLF productionLF = new ProductionLF();
					productionLF.lform = goal.lform;
					productionLF.topic = goal.topic;
					log("adding the canned text LF to the WM: " + LFUtils.lfToString(productionLF.lform));
					addToWorkingMemory(newDataID(),productionLF);
				} // if ..else check for canned text
			} // end while
    	}
    	catch (Exception e) {
    		e.printStackTrace();
    		throw new DialogueException(e.getMessage());
    	} // end try..catch overwriting working memory
	} // end executeTask


} // end class
