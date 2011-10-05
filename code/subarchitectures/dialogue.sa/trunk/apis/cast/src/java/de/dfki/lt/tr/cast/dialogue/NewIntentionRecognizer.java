package de.dfki.lt.tr.cast.dialogue;

import cast.SubarchitectureComponentException;
import cast.architecture.ChangeFilterFactory;
import cast.architecture.WorkingMemoryChangeReceiver;
import cast.architecture.WorkingMemoryWriterComponent;
import cast.cdl.WorkingMemoryAddress;
import cast.cdl.WorkingMemoryChange;
import cast.cdl.WorkingMemoryOperation;
import de.dfki.lt.tr.beliefs.slice.epstatus.EpistemicStatus;
import de.dfki.lt.tr.dialogue.interpret.AbducerUtils;
import de.dfki.lt.tr.dialogue.interpret.atoms.AssertedReferenceAtom;
import de.dfki.lt.tr.dialogue.interpret.ConversionUtils;
import de.dfki.lt.tr.dialogue.interpret.IntentionManagementConstants;
import de.dfki.lt.tr.dialogue.interpret.InterpretedUserIntention;
import de.dfki.lt.tr.dialogue.interpret.InterpretedUserIntentionProofInterpreter;
import de.dfki.lt.tr.dialogue.interpret.ReferenceGatherer;
import de.dfki.lt.tr.dialogue.interpret.atoms.FromLFAtom;
import de.dfki.lt.tr.dialogue.interpret.atoms.IntentionIDAtom;
import de.dfki.lt.tr.dialogue.interpret.atoms.NewBeliefAtom;
import de.dfki.lt.tr.dialogue.interpret.atoms.ReferentOfAtom;
import de.dfki.lt.tr.dialogue.ref.BasicReferenceResolutionRequestExtractor;
import de.dfki.lt.tr.dialogue.ref.EpistemicReferenceHypothesis;
import de.dfki.lt.tr.dialogue.ref.ReferenceResolutionRequest;
import de.dfki.lt.tr.dialogue.ref.ReferenceResolutionRequestExtractor;
import de.dfki.lt.tr.dialogue.ref.ReferenceResolutionResult;
import de.dfki.lt.tr.dialogue.ref.util.ReferenceUtils;
import de.dfki.lt.tr.dialogue.slice.interpret.InterpretationRequest;
import de.dfki.lt.tr.dialogue.slice.lf.LogicalForm;
import de.dfki.lt.tr.dialogue.slice.parseselection.SelectedLogicalForm;
import de.dfki.lt.tr.dialogue.time.TimeInterval;
import de.dfki.lt.tr.dialogue.util.LFUtils;
import de.dfki.lt.tr.infer.abducer.engine.AbductionEnginePrx;
import de.dfki.lt.tr.infer.abducer.engine.FileReadErrorException;
import de.dfki.lt.tr.infer.abducer.engine.SyntaxErrorException;
import de.dfki.lt.tr.infer.abducer.lang.DisjointDeclaration;
import de.dfki.lt.tr.infer.abducer.lang.FunctionTerm;
import de.dfki.lt.tr.infer.abducer.lang.ModalisedAtom;
import de.dfki.lt.tr.infer.abducer.lang.Modality;
import de.dfki.lt.tr.infer.abducer.lang.Term;
import de.dfki.lt.tr.infer.abducer.proof.AbstractAssertionSolver;
import de.dfki.lt.tr.infer.abducer.proof.AbstractProofInterpretationContext;
import de.dfki.lt.tr.infer.abducer.proof.AssertionSolverCascade;
import de.dfki.lt.tr.infer.abducer.proof.ContextUpdate;
import de.dfki.lt.tr.infer.abducer.proof.EngineProofExpander;
import de.dfki.lt.tr.infer.abducer.proof.ProofExpander;
import de.dfki.lt.tr.infer.abducer.proof.ProofInterpretationContext;
import de.dfki.lt.tr.infer.abducer.proof.ProofInterpreter;
import de.dfki.lt.tr.infer.abducer.proof.ProofPruner;
import de.dfki.lt.tr.infer.abducer.proof.pruners.LengthPruner;
import de.dfki.lt.tr.infer.abducer.util.AbductionEngineConnection;
import de.dfki.lt.tr.infer.abducer.util.PrettyPrint;
import de.dfki.lt.tr.infer.abducer.util.TermAtomFactory;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.HashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;
import java.util.Set;
import java.util.concurrent.ExecutionException;

public class NewIntentionRecognizer
extends AbstractInterpretationManager<InterpretedUserIntention> {

	public final String DEFAULT_ABD_SERVER_NAME = "AbducerServer";
	public final int DEFAULT_ABD_PORT = 9100;
	public final String DEFAULT_ABD_ENDPOINT_CONFIG = "default";
	public final String DEFAULT_ENGINE_NAME = "intention-recognition";

	private String abd_serverName = DEFAULT_ABD_SERVER_NAME;
	private List<String> files = new LinkedList<String>();
	private String abd_endpoints;
	private String abd_engineName = DEFAULT_ENGINE_NAME;

	private String rulesetFile = "/dev/null";

	private int timeout = 250;

	private final Map<String, SelectedLogicalForm> nomToLFMap;

	private final ReferenceResolutionRequestExtractor rrExtractor;
	private final Map<WorkingMemoryAddress, ReferenceGatherer> gatherers;

	public NewIntentionRecognizer() {
		super();
		nomToLFMap = new HashMap<String, SelectedLogicalForm>();
		rrExtractor = new BasicReferenceResolutionRequestExtractor(getLogger());
		gatherers = new HashMap<WorkingMemoryAddress, ReferenceGatherer>();
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

		String abd_host = args.get("--abd-host");
		abd_endpoints = AbducerUtils.getAbducerServerEndpointString(abd_host, DEFAULT_ABD_PORT);

		files.addAll(AbducerUtils.getAbducerRuleFiles(getLogger(), rulesetFile));
	}

	@Override
	public AbductionEnginePrx initEngine() {
		getLogger().debug("initialising the abduction engine, servername=\"" + abd_serverName + "\", endpoints=\"" + abd_endpoints + "\"");
		AbductionEngineConnection connection = new AbductionEngineConnection();
		connection.connectToServer(abd_serverName, abd_endpoints);
		getLogger().debug("binding to the engine \"" + abd_engineName + "\"");
		connection.bindToEngine(abd_engineName);

		AbductionEnginePrx prx = connection.getEngineProxy();
		prx.clearContext();

		for (String filename : files) {
			try {
				prx.loadFile(filename);
			}
			catch (FileReadErrorException ex) {
				logException(ex);
			}
			catch (SyntaxErrorException ex) {
				logException(ex);
			}
		}

		return prx;
	}

	@Override
	protected ProofInterpretationContext<InterpretedUserIntention> initContext() {
		ProofPruner pruner = new LengthPruner(3);
		ProofExpander expander = new EngineProofExpander(getEngine(), timeout);

		AssertionSolverCascade solvers = new AssertionSolverCascade();
		solvers.addSolver(new ExpandLFAssertionSolver());
		solvers.addSolver(new IntentionIDAssertionSolver());
		solvers.addSolver(new NewBeliefAssertionSolver());
		solvers.addSolver(new ReferenceResolutionAssertionSolver());

		ProofInterpreter<InterpretedUserIntention> interpreter = new InterpretedUserIntentionProofInterpreter(getLogger());

		final WorkingMemoryWriterComponent committer = this;

		return new AbstractProofInterpretationContext<InterpretedUserIntention>(pruner, expander, solvers, interpreter) {

			@Override
			public void onSuccessfulInterpretation(InterpretedUserIntention i) {
				try {
					getLogger().debug("going to commit the following:\n" + i.toString());
					i.commit(committer);
				}
				catch (SubarchitectureComponentException ex) {
					logException(ex);
				}
			}

			@Override
			public void onNoInterpretation() {
				getLogger().warn("no interpretation found");
			}
			
		};
	}

	@Override
	public void onStart() {
		super.onStart();

		addChangeFilter(ChangeFilterFactory.createLocalTypeFilter(
				SelectedLogicalForm.class, WorkingMemoryOperation.ADD),
				new WorkingMemoryChangeReceiver() {
					@Override
					public void workingMemoryChanged(WorkingMemoryChange _wmc) {
						addTask(new ProcessingTaskWithData<WorkingMemoryAddress>(_wmc.address) {

							@Override
							public void execute(WorkingMemoryAddress addr) {
								try {
									getLogger().info("converting the SelectedLogicalForm to an InterpretationRequest");
									SelectedLogicalForm slf = getMemoryEntry(addr, SelectedLogicalForm.class);
									nomToLFMap.put(slf.lform.root.nomVar, slf);
									InterpretationRequest inprRequest = new InterpretationRequest(selectedLFToGoal(slf));
									addToWorkingMemory(newDataID(), inprRequest);
								}
								catch (SubarchitectureComponentException ex) {
									logException(ex);
								}
							}
							
						});
					}
				});

		addChangeFilter(ChangeFilterFactory.createGlobalTypeFilter(
				ReferenceResolutionResult.class, WorkingMemoryOperation.ADD),
				new WorkingMemoryChangeReceiver() {
					@Override
					public void workingMemoryChanged(WorkingMemoryChange _wmc) {
						try {
							getLogger().info("got a ReferenceResolutionResult");
							ReferenceResolutionResult result = getMemoryEntry(_wmc.address, ReferenceResolutionResult.class);
							ReferenceGatherer gatherer = gatherers.get(result.requestAddress);
							if (gatherer != null) {
								gatherer.addResult(result);
							}
							else {
								getLogger().error("gatherer is null!");
							}
						}
						catch (SubarchitectureComponentException ex) {
							logException(ex);
						}
					}
				});
	}

	public class ExpandLFAssertionSolver extends AbstractAssertionSolver<FromLFAtom> {

		public ExpandLFAssertionSolver() {
			super(new FromLFAtom.Matcher());
		}

		@Override
		public ContextUpdate solveFromParsed(FromLFAtom a) {
			String nominal = a.getNominal();
			getLogger().info("solving an LF assertion for nominal \"" + nominal + "\"");
			final LogicalForm lf = nomToLFMap.get(nominal).lform;
			return new ContextUpdate() {

				@Override
				public void doUpdate(AbductionEnginePrx engine) {
					getLogger().debug("updating the abduction context for the solved LF assertion");
					for (ModalisedAtom fact : AbducerUtils.lfToFacts(new Modality[] {Modality.Truth}, lf)) {
						engine.addFact(fact);
					}
				}

			};
		}

	};

	public class IntentionIDAssertionSolver extends AbstractAssertionSolver<IntentionIDAtom> {

		public IntentionIDAssertionSolver() {
			super(new IntentionIDAtom.Matcher());
		}

		@Override
		public ContextUpdate solveFromParsed(IntentionIDAtom a) {
			final String nominal = a.getNominal();
			if (nominal == null) {
				getLogger().error("undetermined nominal");
				return null;
			}

			final WorkingMemoryAddress wma = newWorkingMemoryAddress();

			getLogger().info("generated a new intention WMA: " + wmaToString(wma));
			return new ContextUpdate() {

				@Override
				public void doUpdate(AbductionEnginePrx engine) {
					ModalisedAtom newFact = TermAtomFactory.modalisedAtom(
							new Modality[] {
								Modality.Understanding,
								Modality.Truth
							},
							TermAtomFactory.atom("intention", new Term[] {
								TermAtomFactory.term(nominal),
								ConversionUtils.workingMemoryAddressToTerm(wma)
							}));
					getLogger().debug("adding the ID to the abduction context: " + PrettyPrint.modalisedAtomToString(newFact));
					engine.addFact(newFact);
				}
				
			};
		}

	};

	public class NewBeliefAssertionSolver extends AbstractAssertionSolver<NewBeliefAtom> {

		public NewBeliefAssertionSolver() {
			super(new NewBeliefAtom.Matcher());
		}

		@Override
		public ContextUpdate solveFromParsed(NewBeliefAtom a) {
//			if (a.getIntentionAddress() == null) {
//				getLogger().error("intention address is null");
//				return null;
//			}
			if (a.getBeliefAddress() != null) {
				getLogger().error("working memory address is already set");
				return null;
			}

			WorkingMemoryAddress newWma = newWorkingMemoryAddress();
			final NewBeliefAtom newAtom = new NewBeliefAtom(a.getIntentionAddress(), newWma, a.getEpistemicStatus());

			getLogger().info("generated a new belief WMA: " + wmaToString(newWma));

			return new ContextUpdate() {

				@Override
				public void doUpdate(AbductionEnginePrx engine) {
					ModalisedAtom matom = newAtom.toModalisedAtom();
					getLogger().debug("adding the ID to the context: " + PrettyPrint.modalisedAtomToString(matom));
					engine.addFact(matom);
				}
				
			};
		}

	}

	public class ReferenceResolutionAssertionSolver extends AbstractAssertionSolver<AssertedReferenceAtom> {

		public ReferenceResolutionAssertionSolver() {
			super(new AssertedReferenceAtom.Matcher());
		}

		@Override
		public ContextUpdate solveFromParsed(AssertedReferenceAtom a) {
			String nominal = a.getNominal();
			if (nominal == null) {
				getLogger().error("nominal undetermined");
				return null;
			}

			SelectedLogicalForm slf = findSelectedLogicalFormByContainedNominal(nominal);
			if (slf == null) {
				getLogger().error("no LF containing " + nominal + "found");
				return null;
			}

			ReferenceResolutionRequest rr = rrExtractor.extractReferenceResolutionRequest(slf.lform, nominal, new TimeInterval(slf.ival));
			if (rr == null) {
				getLogger().error("extracted ReferenceResolutionRequest is null");
				return null;
			}

			WorkingMemoryAddress wma = new WorkingMemoryAddress(newDataID(), getSubarchitectureID());
			ReferenceGatherer gatherer = new ReferenceGatherer(wma, rr);
			gatherers.put(wma, gatherer);

			ReferenceResolutionResult result = null;

			try {
				getLogger().info("adding a ReferenceResolutionRequest to the WM: " + ReferenceUtils.resolutionRequestToString(rr));
				addToWorkingMemory(wma, rr);
				
				// TODO register this as added?
				result = gatherer.getResultFuture().get();
			}
			catch (SubarchitectureComponentException ex) {
				logException(ex);
			}
			catch (InterruptedException ex) {
				logException(ex);
			}
			catch (ExecutionException ex) {
				logException(ex);
			}

			if (result != null) {
				final ReferenceResolutionResult refs = result;
				getLogger().info("got a reference resolution result:\n" + ReferenceUtils.resolutionResultToString(result));
				return new ContextUpdate() {

					@Override
					public void doUpdate(AbductionEnginePrx engine) {
						getLogger().info("updating reference resolution");

						engine.clearAssumabilityFunction("reference_resolution");
						Map<String, Set<ModalisedAtom>> disj = new HashMap<String, Set<ModalisedAtom>>();

						String nom = refs.nom;
						for (EpistemicReferenceHypothesis hypo : refs.hypos) {

							ModalisedAtom rma = ReferentOfAtom.newReferentOfAtom(nom, hypo.referent, hypo.epst).toModalisedAtom();

							getLogger().debug("adding reference hypothesis: " + PrettyPrint.modalisedAtomToString(rma) + " @ p=" + hypo.score);
							engine.addAssumable("reference_resolution", rma, (float) -Math.log(hypo.score));

							Set<ModalisedAtom> dj = disj.get(nom);
							if (dj != null) {
								dj.add(rma);
							}
							else {
								dj = new HashSet<ModalisedAtom>();
								dj.add(rma);
							}
							disj.put(nom, dj);
						}

						for (String n : disj.keySet()) {
							Set<ModalisedAtom> dj = disj.get(n);
							DisjointDeclaration dd = new DisjointDeclaration();
							dd.atoms = new ArrayList<ModalisedAtom>(dj);
							getLogger().debug("adding a disjoint declaration for " + n + " (" + dj.size() + " entries)");
							engine.addDisjointDeclaration(dd);
						}
						getLogger().info("done context update for reference resolution");
					}

				};
			}
			else {
				getLogger().error("result was null");
				return null;
			}
		}

	};

	public static ModalisedAtom selectedLFToGoal(SelectedLogicalForm slf) {
		LogicalForm lf = slf.lform;

		ModalisedAtom goal = TermAtomFactory.modalisedAtom(
				new Modality[] {
					Modality.Understanding,
					Modality.Event
				},
				TermAtomFactory.atom("utter", new Term[] {
					TermAtomFactory.term(IntentionManagementConstants.humanAgent),
					TermAtomFactory.term(IntentionManagementConstants.thisAgent),
					TermAtomFactory.term(lf.root.nomVar)
				}));

		return goal;
	}

	protected WorkingMemoryAddress newWorkingMemoryAddress() {
		return new WorkingMemoryAddress(newDataID(), getSubarchitectureID());
	}

	protected SelectedLogicalForm findSelectedLogicalFormByContainedNominal(String nominal) {
		SelectedLogicalForm result = null;
		for (SelectedLogicalForm slf : nomToLFMap.values()) {
			LogicalForm lf = slf.lform;
			if (LFUtils.lfHasNomvar(lf, nominal)) {
				return slf;
			}
		}
		return result;
	}

}
