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
import cast.UnknownSubarchitectureException;
import cast.architecture.ChangeFilterFactory;
import cast.architecture.WorkingMemoryChangeReceiver;
import cast.cdl.WorkingMemoryChange;
import cast.cdl.WorkingMemoryOperation;
import cast.core.CASTData;
import de.dfki.lt.tr.beliefs.slice.epobject.EpistemicObject;
import de.dfki.lt.tr.beliefs.slice.epstatus.AttributedEpistemicStatus;
import de.dfki.lt.tr.beliefs.slice.epstatus.SharedEpistemicStatus;
import de.dfki.lt.tr.beliefs.slice.intentions.CommunicativeIntention;
import de.dfki.lt.tr.beliefs.slice.intentions.Intention;
import de.dfki.lt.tr.beliefs.slice.sitbeliefs.dBelief;
import de.dfki.lt.tr.cast.ProcessingData;
import de.dfki.lt.tr.dialogue.interpret.IntentionRecognition;
import de.dfki.lt.tr.dialogue.interpret.BeliefIntentionUtils;
import de.dfki.lt.tr.dialogue.interpret.RecognisedIntention;
import de.dfki.lt.tr.dialogue.slice.lf.LogicalForm;
import de.dfki.lt.tr.dialogue.slice.ref.RefLogicalForm;
import de.dfki.lt.tr.dialogue.util.DialogueException;
import de.dfki.lt.tr.dialogue.util.IdentifierGenerator;
import eu.cogx.beliefs.slice.AssertedBelief;
import eu.cogx.beliefs.slice.PresupposedBelief;
import java.io.BufferedReader;
import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileReader;
import java.io.IOException;
import java.util.HashMap;
import java.util.Iterator;
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
public class CommunicativeIntentionRecognizer
extends AbstractDialogueComponent {

	public final static int DEFAULT_TIMEOUT = 250;
	private int timeout = DEFAULT_TIMEOUT;

	private IntentionRecognition irecog;
	private String rulesetFile = "/dev/null";
	private HashMap<String, EpistemicObject> epObjs = new HashMap<String, EpistemicObject>();

	@Override
	public void start() {
		super.start();

		irecog = new IntentionRecognition(new IdentifierGenerator() {
			@Override
			public String newIdentifier() {
				return newDataID();
			}
		}, timeout);

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
					irecog.loadFile(file);
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
				ChangeFilterFactory.createLocalTypeFilter(RefLogicalForm.class, WorkingMemoryOperation.ADD),
				new WorkingMemoryChangeReceiver() {
					@Override
					public void workingMemoryChanged(WorkingMemoryChange _wmc) {
						handleResolvedLogicalForm(_wmc);
					}
		});
	}

	@Override
	public void configure(Map<String, String> _config)
	{
		if (_config.containsKey("--ruleset")) {
			rulesetFile = _config.get("--ruleset");
		}
		if (_config.containsKey("--timeout")) {
			String timeoutStr = _config.get("--timeout");
			timeout = Integer.parseInt(timeoutStr);
		}
	}

	private void handleResolvedLogicalForm(WorkingMemoryChange _wmc) {
		try {
			CASTData data = getWorkingMemoryEntry(_wmc.address.id);
			RefLogicalForm arg = (RefLogicalForm)data.getData();
			String taskID = newTaskID();
			ProcessingData pd = new ProcessingData(newProcessingDataId());
			pd.add(data);
			m_proposedProcessing.put(taskID, pd);
			String taskGoal = DialogueGoals.INTENTION_RECOGNITION_TASK;
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

			if (body instanceof RefLogicalForm) {
				RefLogicalForm rlf = (RefLogicalForm) body;
				LogicalForm lf = rlf.lform;
				irecog.updateReferentialHypotheses(rlf.refs);
				RecognisedIntention eos = irecog.logicalFormToEpistemicObjects(lf);
				if (eos != null) {
					log("recognised " + eos.ints.size() + " intentions and " + (eos.pre.size() + eos.post.size()) + " beliefs");
					for (dBelief b : eos.pre) {
						log("adding belief " + b.id + " to binder WM:\n" + BeliefIntentionUtils.beliefToString(b));
						try {
							dBelief db = upCastPrecondition(b);
							addToWorkingMemory(b.id, "binder", db);
						}
						catch (AlreadyExistsOnWMException ex) {
							ex.printStackTrace();
						}
						catch (UnknownSubarchitectureException ex) {
							ex.printStackTrace();
						}
					}
					for (dBelief b : eos.post) {
						log("adding belief " + b.id + " to dialogue WM:\n" + BeliefIntentionUtils.beliefToString(b));
						try {
							addToWorkingMemory(b.id, b);
						}
						catch (AlreadyExistsOnWMException ex) {
							ex.printStackTrace();
						}
					}
					for (Intention i : eos.ints) {
						log("adding communicative intention " + i.id + " to dialogue WM:\n" + BeliefIntentionUtils.intentionToString(i));
						try {
							CommunicativeIntention cit = new CommunicativeIntention();
							cit.intent = i;
							addToWorkingMemory(newDataID(), cit);
//							addToWorkingMemory(i.id, i);
						}
						catch (AlreadyExistsOnWMException ex) {
							ex.printStackTrace();
						}
					}
				}
				else {
					log("no epistemic object recognised");
				}
			}
		}
		else {
			log("no data for processing");
		}
	}

	public dBelief upCastPrecondition(dBelief b) {
		if (b.estatus instanceof AttributedEpistemicStatus) {
			AssertedBelief ab = new AssertedBelief();
			ab.content = b.content;
			ab.estatus = b.estatus;
			ab.frame = b.frame;
			ab.hist = b.hist;
			ab.id = b.id;
			ab.type = b.type;
			return ab;
		}
		if (b.estatus instanceof SharedEpistemicStatus) {
			PresupposedBelief pb = new PresupposedBelief();
			pb.content = b.content;
			pb.estatus = b.estatus;
			pb.frame = b.frame;
			pb.hist = b.hist;
			pb.id = b.id;
			pb.type = b.type;
			return pb;
		}
		return b;
	}

}
