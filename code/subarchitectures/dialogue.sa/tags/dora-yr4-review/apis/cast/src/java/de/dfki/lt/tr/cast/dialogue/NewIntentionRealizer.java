package de.dfki.lt.tr.cast.dialogue;

import cast.SubarchitectureComponentException;
import cast.architecture.ChangeFilterFactory;
import cast.architecture.WorkingMemoryChangeReceiver;
import cast.architecture.WorkingMemoryWriterComponent;
import cast.cdl.WorkingMemoryAddress;
import cast.cdl.WorkingMemoryChange;
import cast.cdl.WorkingMemoryOperation;
import de.dfki.lt.tr.beliefs.slice.intentions.IntentionToAct;
import de.dfki.lt.tr.dialogue.interpret.AbducerUtils;
import de.dfki.lt.tr.dialogue.interpret.CASTResultWrapper;
import de.dfki.lt.tr.dialogue.interpret.ConversionUtils;
import de.dfki.lt.tr.dialogue.interpret.FirstComeFirstServeCombinator;
import de.dfki.lt.tr.dialogue.interpret.IntentionManagementConstants;
import de.dfki.lt.tr.dialogue.interpret.MaximumReadingsTerminationCondition;
import de.dfki.lt.tr.dialogue.interpret.ResultGatherer;
import de.dfki.lt.tr.dialogue.interpret.RobotCommunicativeAction;
import de.dfki.lt.tr.dialogue.interpret.RobotCommunicativeActionProofInterpreter;
import de.dfki.lt.tr.dialogue.interpret.TerminationCondition;
import de.dfki.lt.tr.dialogue.interpret.atoms.FromIntentionAtom;
import de.dfki.lt.tr.dialogue.interpret.atoms.GenerateReferringExpressionAtom;
import de.dfki.lt.tr.dialogue.production.ProductionUtils;
import de.dfki.lt.tr.dialogue.production.ReferenceGenerationRequest;
import de.dfki.lt.tr.dialogue.production.ReferenceGenerationResult;
import de.dfki.lt.tr.dialogue.slice.interpret.InterpretationRequest;
import de.dfki.lt.tr.infer.abducer.engine.AbductionEnginePrx;
import de.dfki.lt.tr.infer.abducer.engine.FileReadErrorException;
import de.dfki.lt.tr.infer.abducer.engine.SyntaxErrorException;
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
import de.dfki.lt.tr.infer.abducer.util.TermAtomFactory;
import java.util.HashMap;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;
import java.util.concurrent.TimeUnit;

public class NewIntentionRealizer
extends AbstractAbductiveComponent<RobotCommunicativeAction, String> {

	public final String DEFAULT_ABD_SERVER_NAME = "AbducerServer";
	public final int DEFAULT_ABD_PORT = 9100;
	public final String DEFAULT_ABD_ENDPOINT_CONFIG = "default";
	public final String DEFAULT_ENGINE_NAME = "intention-realization";
	public final int DEFAULT_TIMEOUT = 250;

	private String abd_serverName = DEFAULT_ABD_SERVER_NAME;
	private List<String> files = new LinkedList<String>();
	private String abd_endpoints;
	private String abd_engineName = DEFAULT_ENGINE_NAME;

	private String rulesetFile = "/dev/null";
	private final TerminationCondition condition = new MaximumReadingsTerminationCondition(3);

	private int timeout = DEFAULT_TIMEOUT;

	private final Map<WorkingMemoryAddress, IntentionToAct> wmaToInt;
	private final Map<WorkingMemoryAddress, ResultGatherer<ReferenceGenerationResultWrapper>> gatherers;

	public NewIntentionRealizer() {
		super();
		wmaToInt = new HashMap<WorkingMemoryAddress, IntentionToAct>();
		gatherers = new HashMap<WorkingMemoryAddress, ResultGatherer<ReferenceGenerationResultWrapper>>();
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
	protected AbductionEnginePrx initEngine() {
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
	protected ProofInterpretationContext<RobotCommunicativeAction, String> initContext() {
		ProofPruner pruner = new LengthPruner(3);
		ProofExpander expander = new EngineProofExpander(getEngine(), timeout);

		AssertionSolverCascade solvers = new AssertionSolverCascade();
		solvers.addSolver(new ExpandIntentionAssertionSolver());
		solvers.addSolver(new ReferenceGenerationAssertionSolver());
//		solvers.addSolver(new IntentionIDAssertionSolver());
//		solvers.addSolver(new NewBeliefAssertionSolver());
//		solvers.addSolver(new ReferenceResolutionAssertionSolver());

		ProofInterpreter<RobotCommunicativeAction> interpreter = new RobotCommunicativeActionProofInterpreter(getLogger());

		final WorkingMemoryWriterComponent committer = this;

		return new AbstractProofInterpretationContext<RobotCommunicativeAction, String>(pruner, expander, solvers, interpreter) {

			@Override
			public void onSuccessfulInterpretation(List<RobotCommunicativeAction> listIpret, double asrConfidence, String arg) {
				getLogger().debug("got " + listIpret.size() + " interpretations.");
				for (int i = 0; i < listIpret.size(); i++) {
					getLogger().debug("interpretation " + i + "/" + listIpret.size() + ": " + listIpret.get(i));
				}

				if (!listIpret.isEmpty()) {
					getLogger().debug("will now commit the first interpretation");
					RobotCommunicativeAction ipret = listIpret.get(0);
					try {
						ipret.commit(committer);
					}
					catch (SubarchitectureComponentException ex) {
						logException(ex);
					}
				}
				else {
					getLogger().warn("didn't get any interpretations at all!");
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

		addChangeFilter(ChangeFilterFactory.createGlobalTypeFilter(
				IntentionToAct.class, WorkingMemoryOperation.ADD),
				new WorkingMemoryChangeReceiver() {
					@Override
					public void workingMemoryChanged(WorkingMemoryChange _wmc) {
						addTask(new ProcessingTaskWithData<WorkingMemoryAddress>(_wmc.address) {

							@Override
							public void execute(WorkingMemoryAddress addr) {
								try {
									getLogger().info("converting the IntentionToAct to a partial interpretation");
									IntentionToAct actint = getMemoryEntry(addr, IntentionToAct.class);
									wmaToInt.put(addr, actint);
									InterpretationRequest inprRequest = new InterpretationRequest(intentionToActToGoal(actint, addr));
									addNewPartialInterpretation(addr, interpretationRequestToPartialInterpretation(getContext().getPruner(), addr, inprRequest, 1.0, "", condition));
								}
								catch (SubarchitectureComponentException ex) {
									logException(ex);
								}
							}
							
						});
					}
				});

		addChangeFilter(ChangeFilterFactory.createGlobalTypeFilter(
				ReferenceGenerationResult.class, WorkingMemoryOperation.ADD),
				new WorkingMemoryChangeReceiver() {
					@Override
					public void workingMemoryChanged(WorkingMemoryChange _wmc) {
						try {
							getLogger().info("got a ReferenceGenerationResult");
							ReferenceGenerationResult result = getMemoryEntry(_wmc.address, ReferenceGenerationResult.class);
							ResultGatherer<ReferenceGenerationResultWrapper> gatherer = gatherers.get(result.requestAddress);
							if (gatherer != null) {
								gatherer.addResult(new ReferenceGenerationResultWrapper(result));
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
	
	public static ModalisedAtom intentionToActToGoal(IntentionToAct actint, WorkingMemoryAddress wma) {
		ModalisedAtom goal = TermAtomFactory.modalisedAtom(
				new Modality[] {
					Modality.Generation,
					Modality.Event
				},
				TermAtomFactory.atom("utter", new Term[] {
					TermAtomFactory.term(IntentionManagementConstants.thisAgent),
					TermAtomFactory.term(IntentionManagementConstants.humanAgent),
					ConversionUtils.workingMemoryAddressToTerm(wma)
				}));

		return goal;
	}

	public class ExpandIntentionAssertionSolver extends AbstractAssertionSolver<FromIntentionAtom> {

		public ExpandIntentionAssertionSolver() {
			super(new FromIntentionAtom.Matcher());
		}

		@Override
		public ContextUpdate solveFromParsed(FromIntentionAtom a) {
			final WorkingMemoryAddress wma = a.getAddress();
			if (wma == null) {
				return null;
			}
			getLogger().info("solving an intention expansion assertion for " + wmaToString(wma));
			final IntentionToAct actint = wmaToInt.get(wma);
			assert actint != null;

			return new ContextUpdate() {

				@Override
				public void doUpdate(AbductionEnginePrx engine) {
					getLogger().debug("updating the abduction context for the solved intention expansion");
					for (ModalisedAtom fact : RobotCommunicativeAction.intentionToActToFacts(wma, actint)) {
						engine.addFact(fact);
					}
				}

			};
		}

	};

	public class ReferenceGenerationAssertionSolver extends AbstractAssertionSolver<GenerateReferringExpressionAtom> {

		public ReferenceGenerationAssertionSolver() {
			super(new GenerateReferringExpressionAtom.Matcher());
		}

		@Override
		public ContextUpdate solveFromParsed(GenerateReferringExpressionAtom a) {
			final WorkingMemoryAddress beliefAddr = a.getBeliefAddress();
			if (beliefAddr == null) {
				return null;
			}

			final ReferenceGenerationRequest request = new ReferenceGenerationRequest(beliefAddr, a.getHasShortNP(), a.getHasSpatialRelation(), a.getDisabledProps());
			if (request == null) {
				getLogger().error("extracted ReferenceGenerationRequest is null");
				return null;
			}

			WorkingMemoryAddress wma = new WorkingMemoryAddress(newDataID(), getSubarchitectureID());
			ResultGatherer<ReferenceGenerationResultWrapper> gatherer = new ResultGatherer(wma, new FirstComeFirstServeCombinator<ReferenceGenerationResultWrapper>());
			gatherers.put(wma, gatherer);

			ReferenceGenerationResultWrapper result = null;

			try {
				getLogger().info("adding a ReferenceGenerationRequest to the WM: " + ProductionUtils.referenceGenerationRequestToString(request));
				addToWorkingMemory(wma, request);
			}
			catch (SubarchitectureComponentException ex) {
				logException(ex);
			}
			result = gatherer.ensureStabilization(5, TimeUnit.SECONDS);
			stopGathererObservation(wma);


			if (result != null) {
				final ReferenceGenerationResult refs = result.getResult();
				getLogger().info("got a reference generation result:\n" + ProductionUtils.referenceGenerationResultToString(refs));
				return new ContextUpdate() {

					@Override
					public void doUpdate(AbductionEnginePrx engine) {
						getLogger().info("updating the context for the generated RE");

						GenerateReferringExpressionAtom greAtom = new GenerateReferringExpressionAtom(
								beliefAddr,
								request.shortNP,
								request.spatialRelation,
								refs.refEx,
								request.disabledProperties);

						ModalisedAtom matom = greAtom.toModalisedAtom();
						engine.addFact(matom);
					}

				};
			}
			else {
				getLogger().error("result was null");
				return null;
			}

		}
		
	};


	public void stopGathererObservation(WorkingMemoryAddress wma) {
		gatherers.remove(wma);
		try {
			deleteFromWorkingMemory(wma);
		}
		catch (SubarchitectureComponentException ex) {
			logException(ex);
		}
	}

	private static class ReferenceGenerationResultWrapper extends CASTResultWrapper<ReferenceGenerationResult> {

		public ReferenceGenerationResultWrapper(ReferenceGenerationResult result) {
			super(result);
		}

		@Override
		public WorkingMemoryAddress getRequestAddress() {
			return getResult().requestAddress;
		}
		
	}

}
