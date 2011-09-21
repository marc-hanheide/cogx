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
import java.util.concurrent.ConcurrentHashMap;

import cast.AlreadyExistsOnWMException;
import cast.DoesNotExistOnWMException;
import cast.PermissionException;
import cast.SubarchitectureComponentException;
import cast.UnknownSubarchitectureException;
import cast.architecture.ChangeFilterFactory;
import cast.architecture.WorkingMemoryChangeReceiver;
import cast.cdl.WorkingMemoryAddress;
import cast.cdl.WorkingMemoryChange;
import cast.cdl.WorkingMemoryOperation;
import cast.core.CASTData;
import de.dfki.lt.tr.beliefs.slice.epobject.EpistemicObject;
import de.dfki.lt.tr.beliefs.slice.epstatus.PrivateEpistemicStatus;
import de.dfki.lt.tr.beliefs.slice.framing.AbstractFrame;
import de.dfki.lt.tr.beliefs.slice.intentions.CommunicativeIntention;
import de.dfki.lt.tr.beliefs.slice.intentions.Intention;
import de.dfki.lt.tr.beliefs.slice.intentions.IntentionalContent;
import de.dfki.lt.tr.beliefs.slice.logicalcontent.BinaryOp;
import de.dfki.lt.tr.beliefs.slice.logicalcontent.ComplexFormula;
import de.dfki.lt.tr.beliefs.slice.logicalcontent.ElementaryFormula;
import de.dfki.lt.tr.beliefs.slice.logicalcontent.ModalFormula;
import de.dfki.lt.tr.beliefs.slice.logicalcontent.dFormula;
import de.dfki.lt.tr.beliefs.slice.sitbeliefs.dBelief;
import de.dfki.lt.tr.cast.ProcessingData;
import de.dfki.lt.tr.dialogue.interpret.BasicProofConvertor;
import de.dfki.lt.tr.dialogue.interpret.IntentionManagementConstants;
import de.dfki.lt.tr.dialogue.interpret.IntentionRecognition;
import de.dfki.lt.tr.dialogue.interpret.IntentionRecognitionResult;
import de.dfki.lt.tr.dialogue.interpret.ReferenceUtils;
import de.dfki.lt.tr.dialogue.ref.ResolutionRequest;
import de.dfki.lt.tr.dialogue.ref.ResolutionResult;
import de.dfki.lt.tr.dialogue.slice.discourse.DialogueMove;
import de.dfki.lt.tr.dialogue.slice.interpret.Interpretation;
import de.dfki.lt.tr.dialogue.slice.lf.LogicalForm;
import de.dfki.lt.tr.dialogue.slice.parseselection.SelectedLogicalForm;
import de.dfki.lt.tr.dialogue.time.TimeInterval;
import de.dfki.lt.tr.dialogue.util.BeliefIntentionUtils;
import de.dfki.lt.tr.dialogue.util.DialogueException;
import de.dfki.lt.tr.dialogue.util.IdentifierGenerator;
import de.dfki.lt.tr.dialogue.util.LFUtils;
import de.dfki.lt.tr.infer.abducer.proof.AssertedQuery;
import de.dfki.lt.tr.infer.abducer.proof.ProofWithCost;
import de.dfki.lt.tr.infer.abducer.util.ProofUtils;

/**
 * A CAST component/wrapper of the class IntentionManagement. The component
 * listens for finalised PackingLFs appearing on the working memory for
 * intention recognition, and the robot's I-intentions for intention
 * realisation.
 * 
 * Command-line options:
 * 
 * --ruleset FN ... path to the file FN containing one file name on each line.
 * Those files must be located in the same directory as FN and are loaded by the
 * abducer (please refer to the abducer user manual for their syntax).
 * 
 * @author Miroslav Janicek
 */
public class CommunicativeIntentionRecognizer extends AbstractDialogueComponent {

	public final static int DEFAULT_TIMEOUT = 250;
	private int timeout = DEFAULT_TIMEOUT;

	private IntentionRecognition irecog;
	private String rulesetFile = "/dev/null";
	private String dumpFile = "/tmp/belief-model.abd";
	private List<String> files = new LinkedList<String>();
	private HashMap<String, EpistemicObject> epObjs = new HashMap<String, EpistemicObject>();

	private String abd_serverName = "AbducerServer";
	private int abd_port = 9100;
	private String abd_endpoints = "default -p " + abd_port;

	private ConcurrentHashMap<WorkingMemoryAddress, ResolutionRequest> requested = new ConcurrentHashMap<WorkingMemoryAddress, ResolutionRequest>();
	private ConcurrentHashMap<WorkingMemoryAddress, Interpretation> iprets = new ConcurrentHashMap<WorkingMemoryAddress, Interpretation>();

	@Override
	public void start() {
		super.start();

		irecog = new IntentionRecognition(abd_serverName, abd_endpoints,
				new BasicProofConvertor(this.getLogger(".pconv"),
						new IdentifierGenerator() {
							@Override
							public String newIdentifier() {
								return newDataID();
							}
						}, IntentionManagementConstants.thisAgent, "dialogue",
						"binder"), timeout, this.getLogger(".worker"));

		initialiseContext();
		files.add(dumpFile);

		addChangeFilter(ChangeFilterFactory.createLocalTypeFilter(
				SelectedLogicalForm.class, WorkingMemoryOperation.ADD),
				new WorkingMemoryChangeReceiver() {
					@Override
					public void workingMemoryChanged(WorkingMemoryChange _wmc) {
						handleSelectedLogicalForm(_wmc);
					}
				});

		addChangeFilter(ChangeFilterFactory.createLocalTypeFilter(
				Interpretation.class, WorkingMemoryOperation.WILDCARD),
				new WorkingMemoryChangeReceiver() {
					@Override
					public void workingMemoryChanged(WorkingMemoryChange _wmc) {
						handleInterpretation(_wmc);
					}
				});

		addChangeFilter(ChangeFilterFactory.createLocalTypeFilter(
				ResolutionResult.class, WorkingMemoryOperation.OVERWRITE),
				new WorkingMemoryChangeReceiver() {
					@Override
					public void workingMemoryChanged(WorkingMemoryChange _wmc) {
						handleResolutionResultOverwrite(_wmc);
					}
				});
	}

	@Override
	public void configure(Map<String, String> _config) {
		if (_config.containsKey("--ruleset")) {
			rulesetFile = _config.get("--ruleset");
		}
		if (_config.containsKey("--timeout")) {
			String timeoutStr = _config.get("--timeout");
			timeout = Integer.parseInt(timeoutStr);
		}
		if (_config.containsKey("--dumpfile")) {
			dumpFile = _config.get("--dumpfile");
		}
		
		String abducerHost = _config.get("--abd-host");
		if (abducerHost != null) {
			abd_endpoints = "default -h " + abducerHost + " -p " + abd_port;
		}

		if (rulesetFile != null) {
			try {
				BufferedReader f = new BufferedReader(new FileReader(
						rulesetFile));
				String parentAbsPath = (new File(
						(new File(rulesetFile)).getParent()).getCanonicalPath());
				if (parentAbsPath == null) {
					parentAbsPath = ""; // rulefile is in `/'
				}
				log("will be looking for abducer rulefiles in `"
						+ parentAbsPath + "'");
				String file = null;
				while ((file = f.readLine()) != null) {
					file = parentAbsPath + File.separator + file;
					files.add(file);
				}
				f.close();
			} catch (FileNotFoundException e) {
				log("ruleset filename not found");
			} catch (IOException e) {
				log("I/O exception while reading files from list");
				e.printStackTrace();
			}
		}
	}

	private void handleSelectedLogicalForm(WorkingMemoryChange _wmc) {
		try {
			CASTData<SelectedLogicalForm> data = getMemoryEntryWithData(
					_wmc.address, SelectedLogicalForm.class);
			SelectedLogicalForm arg = data.getData();

			getLogger().info(
					"got a SelectedLogicalForm: "
							+ LFUtils.lfToString(arg.lform));

			String taskID = newTaskID();
			ProcessingData pd = new ProcessingData(newProcessingDataId());
			pd.add(data);
			addProposedTask(taskID, pd);
			String taskGoal = DialogueGoals.INTENTION_RECOGNITION_TASK;
			proposeInformationProcessingTask(taskID, taskGoal);
		} catch (SubarchitectureComponentException ex) {
			getLogger().warn("component exception", ex);
		}
	}

	private void handleResolutionResultOverwrite(WorkingMemoryChange _wmc) {
		if (requested.containsKey(_wmc.address)) {
			try {
				log("got a watched ResolutionResult");
				// ResolutionResult rresult = getMemoryEntry(_wmc.address,
				// ResolutionResult.class);
				CASTData data = getWorkingMemoryEntry(_wmc.address.id);

				// okay, now remove it
				requested.remove(_wmc.address);
				this.deleteFromWorkingMemory(_wmc.address);

				ResolutionResult rresult = (ResolutionResult) data.getData();
				String taskID = newTaskID();
				ProcessingData pd = new ProcessingData(newProcessingDataId());
				pd.add(data);
				addProposedTask(taskID, pd);
				String taskGoal = DialogueGoals.INTENTION_RECOGNITION_TASK;
				proposeInformationProcessingTask(taskID, taskGoal);
			} catch (SubarchitectureComponentException ex) {
				getLogger().warn("component exception", ex);
			}
		}
	}

	private void handleInterpretation(WorkingMemoryChange _wmc) {
		try {
			log("got an WM change for Interpretation: (" + _wmc.address.id
					+ "): " + _wmc.operation.toString());
			if (_wmc.operation == WorkingMemoryOperation.DELETE) {
				iprets.remove(_wmc.address);
			} else {
				CASTData data = getWorkingMemoryEntry(_wmc.address.id);

				Interpretation ipret = (Interpretation) data.getData();
				iprets.put(_wmc.address, ipret);

				String taskID = newTaskID();
				ProcessingData pd = new ProcessingData(newProcessingDataId());
				pd.add(data);
				pd.setWorkingMemoryAddress(_wmc.address);
				addProposedTask(taskID, pd);
				String taskGoal = DialogueGoals.INTENTION_RECOGNITION_TASK;
				proposeInformationProcessingTask(taskID, taskGoal);
			}
		} catch (SubarchitectureComponentException ex) {
			ex.printStackTrace();
		}
	}

	private void reexamineInterpretations(ResolutionResult rr) {
		log("reexamining interpretations");

		irecog.updateReferenceResolution(rr);

		List<WorkingMemoryAddress> toRemove = new LinkedList<WorkingMemoryAddress>();

		for (WorkingMemoryAddress wma : iprets.keySet()) {
			IntentionRecognitionResult irr = IntentionRecognitionResult
					.extractFromInterpretation(irecog.getProofConvertor(),
							iprets.get(wma), this.getLogger());
			iprets.get(wma);

			String nom = rr.nom;

			if (irr.getUngroundedNominals().remove(nom)) {
				log("interpretation of (" + wma.id + ") might have changed");

				// TODO: add resolution result to the proofs
				IntentionRecognitionResult new_irr = irecog
						.reinterpret(irr, rr);

				if (new_irr != null) {
					log("scheduling (" + wma.id + ") for processing");
					irr = new_irr;

					Interpretation ipret = irr.toInterpretation();
					CASTData<Interpretation> data = new CASTData(wma.id, ipret); // TODO:
																					// what
																					// is
																					// the
																					// id
																					// here?
					String taskID = newTaskID();
					ProcessingData pd = new ProcessingData(
							newProcessingDataId());
					pd.add(data);
					pd.setWorkingMemoryAddress(wma);
					addProposedTask(taskID, pd);
					String taskGoal = DialogueGoals.INTENTION_RECOGNITION_TASK;
					proposeInformationProcessingTask(taskID, taskGoal);
				} else {
					log("scheduling (" + wma.id + ") for removal");
					toRemove.add(wma);
				}
			}
		}

		for (WorkingMemoryAddress wma : toRemove) {
			try {
				log("removing Interpretation (" + wma.id + ")");
				deleteFromWorkingMemory(wma);
			} catch (DoesNotExistOnWMException ex) {
				ex.printStackTrace();
			} catch (PermissionException ex) {
				ex.printStackTrace();
			} catch (UnknownSubarchitectureException ex) {
				ex.printStackTrace();
			}
		}
	}

	private boolean isUngroundable(Interpretation ipret) {
		if (!ipret.proofs.isEmpty()) {
			ProofWithCost pwc = ipret.proofs.get(0);
			List<AssertedQuery> asserts = ProofUtils.filterAsserted(pwc.proof);
			for (AssertedQuery aq : asserts) {
				if (aq.atom.a.predSym.equals("ungroundable")) {
					return true;
				}
			}
		}
		return false;
	}

	@Override
	public void executeTask(ProcessingData data) throws DialogueException {

		Iterator<CASTData> iter = data.getData();
		if (iter.hasNext()) {
			Object body = iter.next().getData();

			if (body instanceof SelectedLogicalForm) {
				SelectedLogicalForm slf = (SelectedLogicalForm) body;
				LogicalForm lf = slf.lform;
				initialiseContext();
				// irecog.updateReferentialHypotheses(slf.refs);
				IntentionRecognitionResult ri = irecog
						.logicalFormToInterpretation(lf, new TimeInterval(
								slf.ival));
				if (ri != null) {

					Interpretation ipret = ri.toInterpretation();
					try {
						String id = newDataID();
						log("writing the interpretation (" + id + ") to the WM");
						addToWorkingMemory(id, ipret);
					} catch (AlreadyExistsOnWMException ex) {
						ex.printStackTrace();
					}

					log("found " + ri.getResolutionRequests().size()
							+ " references to be resolved");
					for (ResolutionRequest rr : ri.getResolutionRequests()) {
						try {
							log("requesting recognition of this reference:\n"
									+ ReferenceUtils
											.resolutionRequestToString(rr));
							WorkingMemoryAddress wma = new WorkingMemoryAddress(
									newDataID(), this.getSubarchitectureID());
							addToWorkingMemory(wma, rr);
							requested.put(wma, rr);
						} catch (DoesNotExistOnWMException ex) {
							ex.printStackTrace();
						} catch (UnknownSubarchitectureException ex) {
							ex.printStackTrace();
						} catch (AlreadyExistsOnWMException ex) {
							ex.printStackTrace();
						}
					}
				}
			}

			if (body instanceof Interpretation) {
				assert (data.getWorkingMemoryAddress() != null);

				log("processing an Interpretation ("
						+ data.getWorkingMemoryAddress().id + ")");
				Interpretation ipret = (Interpretation) body;

				if (ipret.ungroundedNoms.isEmpty() && !hasAssertions(ipret)) {
					log("hurray! this is a grounded interpretation");
					IntentionRecognitionResult ri = IntentionRecognitionResult
							.extractFromInterpretation(
									irecog.getProofConvertor(), ipret,
									this.getLogger());

					// we won't track it any longer
					WorkingMemoryAddress wma = data.getWorkingMemoryAddress();
					try {
						log("removing the interpretation (" + wma.id
								+ ") from the WM");
						deleteFromWorkingMemory(wma);
					} catch (DoesNotExistOnWMException ex) {
						ex.printStackTrace();
					} catch (PermissionException ex) {
						ex.printStackTrace();
					} catch (UnknownSubarchitectureException ex) {
						ex.printStackTrace();
					}

					log("recognised "
							+ ri.getIntentions().size()
							+ " intentions and "
							+ (ri.getPreconditionBeliefs().size() + ri
									.getPostconditionBeliefs().size())
							+ " beliefs");
					for (dBelief b : ri.getPreconditionBeliefs()) {
						log("adding belief " + b.id + " to binder WM:\n"
								+ BeliefIntentionUtils.beliefToString(b));
						try {
							addToWorkingMemory(b.id, "binder", b);
						} catch (AlreadyExistsOnWMException ex) {
							ex.printStackTrace();
						} catch (UnknownSubarchitectureException ex) {
							ex.printStackTrace();
						}
					}
					for (dBelief b : ri.getPostconditionBeliefs()) {
						log("adding belief " + b.id + " to dialogue WM:\n"
								+ BeliefIntentionUtils.beliefToString(b));
						try {
							addToWorkingMemory(b.id, b);
						} catch (AlreadyExistsOnWMException ex) {
							ex.printStackTrace();
						}
					}
					for (Intention i : ri.getIntentions()) {
						log("adding communicative intention " + i.id
								+ " to dialogue WM:\n"
								+ BeliefIntentionUtils.intentionToString(i));
						try {
							CommunicativeIntention cit = new CommunicativeIntention();
							cit.intent = i;
							addToWorkingMemory(newDataID(), cit);
							// addToWorkingMemory(i.id, i);
						} catch (AlreadyExistsOnWMException ex) {
							ex.printStackTrace();
						}
					}

					if (ri.getNominalReference() == null) {
						log("the communication act does not specify topic");
					} else {
						log("topic: ("
								+ ri.getNominalReference().nominal
								+ ", "
								+ BeliefIntentionUtils.dFormulaToString(ri
										.getNominalReference().referent) + ")");
					}

					// register the dialogue move
					DialogueMove dm = new DialogueMove(
							IntentionManagementConstants.humanAgent,
							ipret.lform, ri.getNominalReference());
					try {
						addToWorkingMemory(newDataID(), dm);
					} catch (AlreadyExistsOnWMException ex) {
						ex.printStackTrace();
					}
				} else {
					if (isUngroundable(ipret)) {
						log("interpretation ungroundable");
						/*
						 * try { log("removing interpretation " +
						 * data.getWorkingMemoryAddress().id + " from WM");
						 * deleteFromWorkingMemory
						 * (data.getWorkingMemoryAddress()); } catch
						 * (DoesNotExistOnWMException ex) {
						 * ex.printStackTrace(); } catch (PermissionException
						 * ex) { ex.printStackTrace(); } catch
						 * (UnknownSubarchitectureException ex) {
						 * ex.printStackTrace(); }
						 */
						log("will generate a notification");
						CommunicativeIntention cit = generateUngroundableIntention(ipret);
						if (cit != null) {
							try {
								addToWorkingMemory(newDataID(), cit);
							} catch (AlreadyExistsOnWMException ex) {
								ex.printStackTrace();
							}
						} else {
							log("failed to generate the notification!");
						}

					} else {
						log("interpretation not grounded yet, will wait for it");
					}
				}
			}

			if (body instanceof ResolutionResult) {
				log("processing a ResolutionResult");
				ResolutionResult rr = (ResolutionResult) body;
				reexamineInterpretations(rr);
			}
		} else {
			log("no data for processing");
		}
		log("task done");
	}

	private CommunicativeIntention generateUngroundableIntention(
			Interpretation ipret) {
		Intention it = new Intention();
		it.id = newDataID();
		it.estatus = new PrivateEpistemicStatus("self");
		it.content = new LinkedList<IntentionalContent>();

		// it's the robot's intention
		List<String> ags = new LinkedList<String>();
		ags.add("self");

		// construct the postcondition (the state)
		ComplexFormula inState = new ComplexFormula(0,
				new LinkedList<dFormula>(), BinaryOp.conj);
		inState.forms.add(new ElementaryFormula(0,
				"referring-failure-announced"));

		ModalFormula state = new ModalFormula(0, "state", inState);

		ComplexFormula post = new ComplexFormula(0, new LinkedList<dFormula>(),
				BinaryOp.conj);
		post.forms.add(state);

		IntentionalContent itc = new IntentionalContent(
				ags,
				new ComplexFormula(0, new LinkedList<dFormula>(), BinaryOp.conj),
				post, 1.0f);
		it.content.add(itc);
		it.frame = new AbstractFrame();

		CommunicativeIntention cit = new CommunicativeIntention(it);
		return cit;
	}

	private boolean hasAssertions(Interpretation ipret) {
		if (!ipret.proofs.isEmpty()) {
			ProofWithCost pwc = ipret.proofs.get(0);
			if (!ProofUtils.filterAsserted(pwc.proof).isEmpty()) {
				return true;
			}
		}
		return false;
	}

	private void initialiseContext() {
		log("initialising context");
		irecog.clearContext();
		for (String f : files) {
			log("reading file " + f);
			irecog.loadFile(f);
		}
	}
}
