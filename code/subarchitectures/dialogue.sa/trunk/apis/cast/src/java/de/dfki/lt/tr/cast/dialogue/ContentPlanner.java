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


//CAST
import cast.architecture.ChangeFilterFactory;
import cast.architecture.WorkingMemoryChangeReceiver;
import cast.cdl.WorkingMemoryAddress;
import cast.cdl.WorkingMemoryChange;
import cast.cdl.WorkingMemoryOperation;

// Dialogue API Slice
import de.dfki.lt.tr.dialogue.slice.produce.ContentPlanningGoal;
import de.dfki.lt.tr.dialogue.slice.produce.ProductionLF;

//Dialogue API CAST

// Dialogue API
import de.dfki.lt.tr.dialogue.util.LFUtils;
import de.dfki.lt.tr.dialogue.cplanwrapper.CPlanWrapper;
import de.dfki.lt.tr.dialogue.slice.lf.LFNominal;
import de.dfki.lt.tr.dialogue.slice.lf.LFRelation;
import de.dfki.lt.tr.dialogue.slice.lf.LogicalForm;

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
	private CPlanWrapper planner;
	private	String contentRel;
	private String cannedTextRel;

	public ContentPlanner() {
		planner = null;
		contentRel = null;
		cannedTextRel = null;
	}

	@Override
	public void onConfigure(Map<String, String> args) {
		super.onConfigure(args);

		if (args.containsKey("--domainFile")) {
			// TODO the second argument should be the path to the directory where
			// the function plugin jar(s) can be found
			File domainFile = new File(args.get("--domainFile"));
			// TODO fix this ! commented out to prevent double trove.jar issues (hz, 2012-04-20)
			// planner = new CPlanWrapper(domainFile, null);
		}
		if (args.containsKey("--contentRel")) {
			contentRel = args.get("--contentRel");
		}
		if (args.containsKey("--cannedText")) {
			cannedTextRel = args.get("--cannedText");
		}
		else {
			cannedTextRel = "CannedText";
		}
	}

	@Override
	public void onStart() {
		super.onStart();
		addChangeFilter(ChangeFilterFactory.createLocalTypeFilter(ContentPlanningGoal.class, WorkingMemoryOperation.ADD),
				new WorkingMemoryChangeReceiver() {
					@Override
					public void workingMemoryChanged(WorkingMemoryChange _wmc) {
						handleContentPlanningGoalAdd(_wmc);
					}
				});
	}


	private void handleContentPlanningGoalAdd(WorkingMemoryChange _wmc) {
		addTask(new ProcessingTaskWithData<WorkingMemoryAddress>(_wmc.address) {

			@Override
			public void execute(WorkingMemoryAddress addr) {
				try {
					ContentPlanningGoal goal = getMemoryEntry(addr, ContentPlanningGoal.class);
					String canned = LFUtils.lfNominalGetFeature(goal.lform.root, cannedTextRel);
					log("Canned text feature value [" + goal.lform.root.nomVar + "]: \"" + canned + "\"");

					if (planner != null && canned.equals("")) {				
						LogicalForm output = planner.callContentPlanner(goal.lform);				
						ProductionLF productionLF = new ProductionLF();

						if (contentRel != null)	{
							try {
								// retrieve the subtree from under the contentRel relation under the root of the LF
								LFRelation crel = LFUtils.lfNominalGetRelation(output.root,contentRel);
								LFNominal croot = LFUtils.lfGetNominal(output,crel.dep);
								LogicalForm content = LFUtils.lfConstructSubtree(croot,output);

								productionLF.lform = content;
							}
							catch (NullPointerException ex) {
								getLogger().error("null in constructing a ProductionLF");
								System.out.println(ex.getStackTrace());
							}
						} else {
							productionLF.lform = output;
						}
						productionLF.plfid = newDataID();
						productionLF.topic = goal.topic;

						getLogger().info("adding the production LF to the WM: " + LFUtils.lfToString(productionLF.lform));
						addToWorkingMemory(newDataID(), productionLF);
					}
					else {
						String id = newDataID();

						ProductionLF productionLF = new ProductionLF(id, goal.lform, goal.topic);

						getLogger().info("adding the canned text LF to the WM: " + LFUtils.lfToString(productionLF.lform));
						addToWorkingMemory(id, productionLF);
					}
				}
				catch (SubarchitectureComponentException ex) {
					getLogger().error("component exception", ex);
				}
			}
		});
	}

}
