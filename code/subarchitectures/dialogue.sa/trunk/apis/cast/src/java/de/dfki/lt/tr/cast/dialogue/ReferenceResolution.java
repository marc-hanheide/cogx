// =================================================================
// Copyright (C) 2010 DFKI GmbH Talking Robots
// Miroslav Janicek (miroslav.janicek@dfki.de)
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

package de.dfki.lt.tr.cast.dialogue;

import cast.AlreadyExistsOnWMException;
import cast.SubarchitectureComponentException;
import cast.architecture.ChangeFilterFactory;
import cast.architecture.WorkingMemoryChangeReceiver;
import cast.cdl.WorkingMemoryChange;
import cast.cdl.WorkingMemoryOperation;
import cast.core.CASTData;
import de.dfki.lt.tr.beliefs.slice.epobject.EpistemicObject;
import de.dfki.lt.tr.beliefs.slice.intentions.Intention;
import de.dfki.lt.tr.beliefs.slice.intentions.IntentionalContent;
import de.dfki.lt.tr.beliefs.slice.sitbeliefs.dBelief;
import de.dfki.lt.tr.cast.ProcessingData;
import de.dfki.lt.tr.dialogue.ref.AbductiveReferenceResolution;
import de.dfki.lt.tr.dialogue.ref.PresupposedBeliefConstruction;
import de.dfki.lt.tr.dialogue.slice.lf.LogicalForm;
import de.dfki.lt.tr.dialogue.slice.parse.PackedLFs;
import de.dfki.lt.tr.dialogue.slice.produce.ContentPlanningGoal;
import de.dfki.lt.tr.dialogue.slice.ref.NominalRef;
import de.dfki.lt.tr.dialogue.slice.ref.RefHypo;
import de.dfki.lt.tr.dialogue.slice.ref.ResolvedLogicalForm;
import de.dfki.lt.tr.dialogue.slice.ref.RefReadings;
import de.dfki.lt.tr.dialogue.util.DialogueException;
import de.dfki.lt.tr.dialogue.util.LFUtils;
import de.dfki.lt.tr.infer.weigabd.AbductionEngineConnection;
import java.io.BufferedReader;
import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileReader;
import java.io.IOException;
import java.util.HashMap;
import java.util.Iterator;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;
import java.util.logging.Level;
import java.util.logging.Logger;

public class ReferenceResolution
extends AbstractDialogueComponent {

/*
	private ReferenceResolution rr;
	private String rulesetFile = "/dev/null";
	private HashMap<String, EpistemicObject> epObjs = new HashMap<String, EpistemicObject>();
 */
	private PresupposedBeliefConstruction pbc;
	private AbductiveReferenceResolution arr;
	private String pbcRulesetFile = "/dev/null";
	private String dumpfile = "/dev/null";
	private String appendfile = "/dev/null";
	private String correlfile = "/dev/null";

	AbductionEngineConnection intentionEngine = null;

	@Override
	public void start() {
		super.start();
		pbc = new PresupposedBeliefConstruction();
		if (pbcRulesetFile != null) {
			try {
				BufferedReader f = new BufferedReader(new FileReader(pbcRulesetFile));
				String parentAbsPath = (new File((new File(pbcRulesetFile)).getParent()).getCanonicalPath());
				if (parentAbsPath == null) {
					parentAbsPath = "";  // rulefile is in `/'
				}
				log("will be looking for abducer rulefiles in `" + parentAbsPath + "'");
				String file = null;
				while ((file = f.readLine()) != null) {
					file = parentAbsPath + File.separator + file;
					log("adding file " + file);
					pbc.loadFile(file);
				}
				f.close();
			}
			catch (FileNotFoundException e) {
				log("ruleset filename not found");
			}
			catch (IOException e) {
				log("I/O exception while reading files from list");
				e.printStackTrace();
			}
		}
		else {
			log("no ruleset to read");
		}

		intentionEngine = new AbductionEngineConnection();
		intentionEngine.connectToServer("AbducerServer", "default -p 10000");
		intentionEngine.bindToEngine("IntentionManagementEngine");

		arr = new AbductiveReferenceResolution(toAbsolutePath(dumpfile), toAbsolutePath(appendfile), intentionEngine);
		if (correlfile != null) {
			arr.loadFile(toAbsolutePath(correlfile));
		}

		addChangeFilter(
				ChangeFilterFactory.createLocalTypeFilter(LogicalForm.class,  WorkingMemoryOperation.ADD),
				new WorkingMemoryChangeReceiver() {
					@Override
					public void workingMemoryChanged(WorkingMemoryChange _wmc) {
						handleLogicalForm(_wmc);
					}
				});
	}

	@Override
	public void configure(Map<String, String> _config)
	{
		if (_config.containsKey("--ruleset-construct")) {
			pbcRulesetFile = _config.get("--ruleset-construct");
		}
		if (_config.containsKey("--appendfile")) {
			appendfile = _config.get("--appendfile");
		}
		if (_config.containsKey("--correlfile")) {
			correlfile = _config.get("--correlfile");
		}
		if (_config.containsKey("--dumpfile")) {
			dumpfile = _config.get("--dumpfile");
		}
	}

	private static String toAbsolutePath(String path) {
		try {
			return (new File(path)).getCanonicalPath();
		}
		catch (IOException ex) {
			ex.printStackTrace();
		}
		return null;
	}

	private void handleLogicalForm(WorkingMemoryChange _wmc) {
		try {
			CASTData data = getWorkingMemoryEntry(_wmc.address.id);
			LogicalForm arg = (LogicalForm)data.getData();
			String taskID = newTaskID();
			ProcessingData pd = new ProcessingData(newProcessingDataId());
			pd.add(data);
			m_proposedProcessing.put(taskID, pd);
			String taskGoal = DialogueGoals.REFERENCE_RESOLUTION_TASK;
			proposeInformationProcessingTask(taskID, taskGoal);
		}
		catch (SubarchitectureComponentException e) {
			e.printStackTrace();
		}
	}

	@Override
	public void executeTask(ProcessingData data)
	throws DialogueException {
		Iterator<CASTData> iter = data.getData();
		if (iter.hasNext()) {
			Object body = iter.next().getData();

			if (body instanceof LogicalForm) {
				LogicalForm lf = (LogicalForm) body;
				Map<String, Map<String, String>> eos = pbc.extractPresuppositions(lf);

				ResolvedLogicalForm rlf = new ResolvedLogicalForm();
				rlf.lform = lf;
				rlf.refs = new NominalRef[0];
				List<NominalRef> nrs = new LinkedList<NominalRef>();

				intentionEngine.getProxy().clearAssumabilityFunction("reference_resolution");
				if (eos != null && !eos.isEmpty()) {
					for (String nom : eos.keySet()) {
						NominalRef nr =	arr.resolvePresupposition(nom, eos.get(nom));
						log(nr.hypos.length + " hypos for ["
								+ PresupposedBeliefConstruction.presupToString(nom, eos.get(nom)) + "]");

/*
						for (RefHypo hypo : nr.hypos) {
							log("  " + hypo.beliefId + " @ p=" + hypo.prob);
						}
 */
					}
					log("reference hypotheses added to the intention recognition engine");
				}
				else {
					log("found no beliefs to resolve");
				}
				rlf.refs = nrs.toArray(new NominalRef[0]);

				try {
					addToWorkingMemory(newDataID(), rlf);
				}
				catch (AlreadyExistsOnWMException ex) {
					ex.printStackTrace();
				}
			}
		}
		else {
			log("no data for processing");
		}
	}

}