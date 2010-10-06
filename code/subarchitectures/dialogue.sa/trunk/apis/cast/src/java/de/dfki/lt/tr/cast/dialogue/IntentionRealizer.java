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
import de.dfki.lt.tr.dialogue.interpret.BeliefIntentionUtils;
import de.dfki.lt.tr.dialogue.interpret.IntentionRealization;
import de.dfki.lt.tr.dialogue.slice.produce.ContentPlanningGoal;
import de.dfki.lt.tr.dialogue.util.DialogueException;
import de.dfki.lt.tr.dialogue.util.IdentifierGenerator;
import de.dfki.lt.tr.dialogue.util.LFUtils;
import java.io.BufferedReader;
import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileReader;
import java.io.IOException;
import java.util.HashMap;
import java.util.Iterator;
import java.util.LinkedList;
import java.util.Map;

/**
 * A CAST component/wrapper of the class IntentionManagement. The component
 * listens for finalised PackingLFs appearing on the working memory for
 * intention recognition, and the robot's I-intentions for intention
 * realisation.
 *
 * Command-line options:
 *
 *   --ruleset FN ...  path to the file FN containing one file name on each
 *                     line. Those files must be located in the same directory
 *                     as FN and are loaded by the abducer (please refer to the
 *                     abducer user manual for their syntax).
 *
 * @author Miroslav Janicek
 */
public class IntentionRealizer
extends AbstractDialogueComponent {

	private IntentionRealization ireal;
	private String rulesetFile = "/dev/null";
	private HashMap<String, EpistemicObject> epObjs = new HashMap<String, EpistemicObject>();

	@Override
	public void start() {
		super.start();

		ireal = new IntentionRealization(new IdentifierGenerator() {
			@Override
			public String newIdentifier() {
				return newDataID();
			}
		});

		if (rulesetFile != null) {
			try {
				BufferedReader f = new BufferedReader(new FileReader(rulesetFile));
				String parentAbsPath = (new File((new File(rulesetFile)).getParent()).getCanonicalPath());
				if (parentAbsPath == null) {
					parentAbsPath = "";  // rulefile is in `/'
				}
				log("will be looking for abducer rulefiles in `" + parentAbsPath + "'");
				String file = null;
				while ((file = f.readLine()) != null) {
					file = parentAbsPath + File.separator + file;
					log("adding file " + file);
					ireal.loadFile(file);
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

		addChangeFilter(
				ChangeFilterFactory.createLocalTypeFilter(Intention.class, WorkingMemoryOperation.ADD),
				new WorkingMemoryChangeReceiver() {
					@Override
					public void workingMemoryChanged(WorkingMemoryChange _wmc) {
						handleIntention(_wmc);
					}
		});
	}

	@Override
	public void configure(Map<String, String> _config)
	{
		if (_config.containsKey("--ruleset")) {
			rulesetFile = _config.get("--ruleset");
		}
	}

	private void handleIntention(WorkingMemoryChange _wmc) {
		try {
			CASTData data = getWorkingMemoryEntry(_wmc.address.id);

			Intention itn = (Intention)data.getData();
			if (itn.content.size() > 1) {
				log("don't know how to handle an intention with " + itn.content.size() + " alternative contents");
				return;
			}
			else {
				IntentionalContent itnc = itn.content.get(0);
				if (itnc.agents.size() == 1 && itnc.agents.get(0).equals("robot")) {
					log("got a private intention, will try to realise it");
					String taskID = newTaskID();
					ProcessingData pd = new ProcessingData(newProcessingDataId());
					pd.add(data);
					m_proposedProcessing.put(taskID, pd);
					String taskGoal = DialogueGoals.INTENTION_REALISATION_TASK;
					proposeInformationProcessingTask(taskID, taskGoal);
				}
			}
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

			if (body instanceof Intention) {
				Intention itn = (Intention) body;
				log("processing an intention");
				LinkedList<String> belIds = BeliefIntentionUtils.collectBeliefIdsInIntention(itn);
				LinkedList<dBelief> bels = new LinkedList<dBelief>();
				for (String id : belIds) {
					dBelief b = retrieveBeliefById(id);
					if (b != null) {
						log("will use belief [" + b.id + "]");
						bels.add(b);
					}
				}

				ContentPlanningGoal protoLF = ireal.epistemicObjectsToProtoLF(itn, bels);
				if (protoLF != null) {
					try {
						log("adding proto-LF to working memory: " + LFUtils.lfToString(protoLF.lform));
						addToWorkingMemory(newDataID(), protoLF);
					}
					catch (Exception e) {
						e.printStackTrace();
						throw new DialogueException(e.getMessage());
					}
				}
				else {
					log("no proto-LF generated");
				}
			}
		}
		else {
			log("no data for processing");
		}
	}

	private dBelief retrieveBeliefById(String id) {
		if (epObjs.containsKey(id)) {
			EpistemicObject eo = epObjs.get(id);
			if (eo instanceof dBelief) {
				return (dBelief)eo;
			}
		}
		return null;
	}

}