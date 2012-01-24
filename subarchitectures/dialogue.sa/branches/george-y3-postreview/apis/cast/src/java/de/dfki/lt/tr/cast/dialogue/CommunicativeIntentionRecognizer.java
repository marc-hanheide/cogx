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
import java.util.LinkedList;
import java.util.List;
import java.util.Map;
import java.util.concurrent.ConcurrentHashMap;

import cast.SubarchitectureComponentException;
import cast.architecture.ChangeFilterFactory;
import cast.architecture.WorkingMemoryChangeReceiver;
import cast.cdl.WorkingMemoryAddress;
import cast.cdl.WorkingMemoryChange;
import cast.cdl.WorkingMemoryOperation;
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
import de.dfki.lt.tr.dialogue.interpret.BasicProofConvertor;
import de.dfki.lt.tr.dialogue.interpret.IntentionManagementConstants;
import de.dfki.lt.tr.dialogue.interpret.IntentionRecognition;
import de.dfki.lt.tr.dialogue.interpret.IntentionRecognitionResult;
import de.dfki.lt.tr.dialogue.ref.BasicReferenceResolutionRequestExtractor;
import de.dfki.lt.tr.dialogue.ref.util.ReferenceUtils;
import de.dfki.lt.tr.dialogue.ref.ReferenceResolutionRequest;
import de.dfki.lt.tr.dialogue.ref.ReferenceResolutionResult;
import de.dfki.lt.tr.dialogue.slice.discourse.DialogueMove;
import de.dfki.lt.tr.dialogue.slice.interpret.Interpretation;
import de.dfki.lt.tr.dialogue.slice.lf.LogicalForm;
import de.dfki.lt.tr.dialogue.slice.parseselection.SelectedLogicalForm;
import de.dfki.lt.tr.dialogue.time.TimeInterval;
import de.dfki.lt.tr.dialogue.util.BeliefIntentionUtils;
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
public class CommunicativeIntentionRecognizer
extends AbstractDialogueComponent {

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

	private ConcurrentHashMap<WorkingMemoryAddress, ReferenceResolutionRequest> requested = new ConcurrentHashMap<WorkingMemoryAddress, ReferenceResolutionRequest>();
	private ConcurrentHashMap<WorkingMemoryAddress, Interpretation> iprets = new ConcurrentHashMap<WorkingMemoryAddress, Interpretation>();

	public CommunicativeIntentionRecognizer() {
		irecog = null;
	}

	@Override
	public void onConfigure(Map<String, String> args) {
		super.onConfigure(args);

		if (args.containsKey("--ruleset")) {
			rulesetFile = args.get("--ruleset");
		}
		if (args.containsKey("--timeout")) {
			String timeoutStr = args.get("--timeout");
			timeout = Integer.parseInt(timeoutStr);
		}
		if (args.containsKey("--dumpfile")) {
			dumpFile = args.get("--dumpfile");
		}
		
		String abducerHost = args.get("--abd-host");
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
			}
			catch (FileNotFoundException ex) {
				getLogger().error("ruleset filename not found", ex);
			}
			catch (IOException ex) {
				getLogger().error("I/O exception while reading files from list", ex);
			}
		}
	}

	@Override
	public void onStart() {
		super.onStart();

		irecog = new IntentionRecognition(abd_serverName, abd_endpoints,
				new BasicProofConvertor(this.getLogger(".pconv"),
						new IdentifierGenerator() {
							@Override
							public String newIdentifier() {
								return newDataID();
							}
						},
						new BasicReferenceResolutionRequestExtractor(this.getLogger(".extractor")),
						IntentionManagementConstants.thisAgent, "dialogue",
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
				ReferenceResolutionResult.class, WorkingMemoryOperation.OVERWRITE),
				new WorkingMemoryChangeReceiver() {
					@Override
					public void workingMemoryChanged(WorkingMemoryChange _wmc) {
						handleResolutionResultOverwrite(_wmc);
					}
				});
	}

	private void handleSelectedLogicalForm(WorkingMemoryChange _wmc) {
		addTask(new ProcessingTaskWithData<WorkingMemoryAddress>(_wmc.address) {

			@Override
			public void execute(WorkingMemoryAddress addr) {
				try {
					SelectedLogicalForm slf = getMemoryEntry(addr, SelectedLogicalForm.class);
					LogicalForm lf = slf.lform;
					getLogger().info("got a SelectedLogicalForm: " + LFUtils.lfToString(lf));

					initialiseContext();
					// irecog.updateReferentialHypotheses(slf.refs);
					IntentionRecognitionResult ri = irecog.logicalFormToInterpretation(lf, new TimeInterval(slf.ival));
					if (ri != null) {

						Interpretation ipret = ri.toInterpretation();
						String id = newDataID();
						log("writing the interpretation (" + id + ") to the WM");
						addToWorkingMemory(id, ipret);

						getLogger().info("found " + ri.getResolutionRequests().size() + " references to be resolved");
						for (ReferenceResolutionRequest rr : ri.getResolutionRequests()) {
							getLogger().info("requesting recognition of this reference:\n" + ReferenceUtils.resolutionRequestToString(rr));
							WorkingMemoryAddress wma = new WorkingMemoryAddress(newDataID(), getSubarchitectureID());
							addToWorkingMemory(wma, rr);
							requested.put(wma, rr);
						}
					}
				}
				catch (SubarchitectureComponentException ex) {
					getLogger().error("component exception", ex);
				}
			}
		});
	}

	private void handleResolutionResultOverwrite(WorkingMemoryChange _wmc) {
		if (requested.containsKey(_wmc.address)) {
			getLogger().debug("got a watched ReferenceResolutionResult, scheduling it for examination");
			addTask(new ProcessingTaskWithData<WorkingMemoryAddress>(_wmc.address) {

				@Override
				public void execute(WorkingMemoryAddress addr) {
					try {
						getLogger().info("processing a ReferenceResolutionResult");
						ReferenceResolutionResult rresult = getMemoryEntry(addr, ReferenceResolutionResult.class);
						reexamineInterpretations(rresult);
					}
					catch (SubarchitectureComponentException ex) {
						getLogger().error("component exception", ex);
					}
				}
			});
		}
	}

	private void handleInterpretation(WorkingMemoryChange _wmc) {
		log("got an WM change for Interpretation: " + wmaToString(_wmc.address) + ": " + _wmc.operation.toString());
		if (_wmc.operation == WorkingMemoryOperation.DELETE) {
			getLogger().info("removing Interpretation " + wmaToString(_wmc.address) + " from the map");
			iprets.remove(_wmc.address);
		}
		else {
			addProcessInterpretationTask(_wmc.address);
		}
	}

	private void addProcessInterpretationTask(WorkingMemoryAddress addr) {
		assert addr != null;
		addTask(new ProcessingTaskWithData<WorkingMemoryAddress>(addr) {

			@Override
			public void execute(WorkingMemoryAddress addr) {
				getLogger().info("processing an Interpretation [" + addr.id + "," + addr.subarchitecture + "]");
				try {
					Interpretation ipret = getMemoryEntry(addr, Interpretation.class);

					if (ipret.ungroundedNoms.isEmpty() && !hasAssertions(ipret)) {
						getLogger().info("hurray! this is a grounded interpretation");
						IntentionRecognitionResult ri = IntentionRecognitionResult.extractFromInterpretation(
												irecog.getProofConvertor(), ipret,
												getLogger());

						// we won't track it any longer
						log("removing the interpretation [" + addr.id + "," + addr.subarchitecture + "] from the WM");
						deleteFromWorkingMemory(addr);

						log("recognised "
								+ ri.getIntentions().size()
								+ " intentions and "
								+ (ri.getPreconditionBeliefs().size() + ri
										.getPostconditionBeliefs().size())
								+ " beliefs");

						for (dBelief b : ri.getPreconditionBeliefs()) {
							log("adding belief " + b.id + " to binder WM:\n"
									+ BeliefIntentionUtils.beliefToString(b));
							addToWorkingMemory(b.id, "binder", b);
						}
						for (dBelief b : ri.getPostconditionBeliefs()) {
							log("adding belief " + b.id + " to dialogue WM:\n"
									+ BeliefIntentionUtils.beliefToString(b));
							addToWorkingMemory(b.id, b);
						}
						for (Intention i : ri.getIntentions()) {
							log("adding communicative intention " + i.id
									+ " to dialogue WM:\n"
									+ BeliefIntentionUtils.intentionToString(i));
							CommunicativeIntention cit = new CommunicativeIntention();
							cit.intent = i;
							addToWorkingMemory(newDataID(), cit);
						}

						if (ri.getNominalReference() == null) {
							log("the communication act does not specify topic");
						}
						else {
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
						addToWorkingMemory(newDataID(), dm);

					}
					else {
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
								addToWorkingMemory(newDataID(), cit);
							}
							else {
								getLogger().warn("failed to generate the notification!");
							}

						} else {
							log("interpretation not grounded yet, will wait for it");
							getLogger().info("adding Interpretation " + wmaToString(addr) + " to the map");
							iprets.put(addr, ipret);
						}
					}
				}
				catch (SubarchitectureComponentException ex) {
					getLogger().error("component exception", ex);
				}
			}
		});
	}

	private void reexamineInterpretations(ReferenceResolutionResult rr) {
		String nominal = rr.nom;
		getLogger().info("reexamining interpretations (looking for nominal " + nominal + " there)");

		irecog.updateReferenceResolution(rr);

		List<WorkingMemoryAddress> toRemove = new LinkedList<WorkingMemoryAddress>();

		String s = "these are the currently ungrounded Interpretations: {\n";
		for (WorkingMemoryAddress wma : iprets.keySet()) {
			Interpretation i = iprets.get(wma);
			s += "  " + wmaToString(wma) + " -> { ";
			for (String nom : i.ungroundedNoms) {
				s += nom + " ";
			}
			s += "}\n";
		}
		s += "}";
		getLogger().debug(s);

		for (WorkingMemoryAddress wma : iprets.keySet()) {
			getLogger().debug("checking whether Interpretation " + wmaToString(wma) + " has changed");
			IntentionRecognitionResult irr = IntentionRecognitionResult.extractFromInterpretation(
					irecog.getProofConvertor(),
					iprets.get(wma),
					this.getLogger());

			iprets.get(wma);

			s = "these are the ungrounded nominals here: { ";
			for (String nom : irr.getUngroundedNominals()) {
				s += nom + " ";
			}
			s += "}";
			getLogger().debug(s);

			if (irr.getUngroundedNominals().remove(nominal)) {
				log("the interpretation of " + wmaToString(wma) + " might have changed");

				// TODO: add resolution result to the proofs
				IntentionRecognitionResult new_irr = irecog.reinterpret(irr, rr);

				if (new_irr != null) {
					log("overwriting " + wmaToString(wma) + " with the new interpretation");
					irr = new_irr;
					Interpretation ipret = irr.toInterpretation();

					try {
						overwriteWorkingMemory(wma, ipret);
					}
					catch (SubarchitectureComponentException ex) {
						getLogger().error("component exception in overwriting an interpretation", ex);
					}
				}
				else {
					log("scheduling " + wmaToString(wma) + " for removal");
					toRemove.add(wma);
				}
			}
			else {
				getLogger().debug("seems that " + wmaToString(wma) + " is unaffected");
			}
		}

		for (WorkingMemoryAddress wma : toRemove) {
			try {
				log("removing Interpretation " + wmaToString(wma));
				deleteFromWorkingMemory(wma);
			}
			catch (SubarchitectureComponentException ex) {
				getLogger().error("component exception in removing an interpretation", ex);
			}
		}
	}

	private static boolean isUngroundable(Interpretation ipret) {
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

	private CommunicativeIntention generateUngroundableIntention(Interpretation ipret) {
		Intention it = new Intention();
		it.id = newDataID();
		it.estatus = new PrivateEpistemicStatus("self");
		it.content = new LinkedList<IntentionalContent>();

		// it's the robot's intention
		List<String> ags = new LinkedList<String>();
		ags.add("self");

		// construct the postcondition (the state)
		ComplexFormula inState = new ComplexFormula(0, new LinkedList<dFormula>(), BinaryOp.conj);
		inState.forms.add(new ElementaryFormula(0, "referring-failure-announced"));

		ModalFormula state = new ModalFormula(0, "state", inState);

		ComplexFormula post = new ComplexFormula(0, new LinkedList<dFormula>(),	BinaryOp.conj);
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

	private static boolean hasAssertions(Interpretation ipret) {
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
