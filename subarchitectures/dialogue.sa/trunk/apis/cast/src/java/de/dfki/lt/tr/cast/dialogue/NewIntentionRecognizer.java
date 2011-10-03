package de.dfki.lt.tr.cast.dialogue;

import cast.SubarchitectureComponentException;
import cast.architecture.ChangeFilterFactory;
import cast.architecture.WorkingMemoryChangeReceiver;
import cast.architecture.WorkingMemoryWriterComponent;
import cast.cdl.WorkingMemoryAddress;
import cast.cdl.WorkingMemoryChange;
import cast.cdl.WorkingMemoryOperation;
import de.dfki.lt.tr.dialogue.interpret.AbducerUtils;
import de.dfki.lt.tr.dialogue.interpret.AssertedReferenceAtom;
import de.dfki.lt.tr.dialogue.interpret.IntentionManagementConstants;
import de.dfki.lt.tr.dialogue.interpret.InterpretedUserIntention;
import de.dfki.lt.tr.dialogue.interpret.InterpretedUserIntentionProofInterpreter;
import de.dfki.lt.tr.dialogue.slice.interpret.InterpretationRequest;
import de.dfki.lt.tr.dialogue.slice.lf.LogicalForm;
import de.dfki.lt.tr.dialogue.slice.parseselection.SelectedLogicalForm;
import de.dfki.lt.tr.infer.abducer.engine.AbductionEnginePrx;
import de.dfki.lt.tr.infer.abducer.engine.FileReadErrorException;
import de.dfki.lt.tr.infer.abducer.engine.SyntaxErrorException;
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
import de.dfki.lt.tr.infer.abducer.util.TermAtomFactory;
import java.util.HashMap;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;

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

	private int timeout;

	private final Map<String, SelectedLogicalForm> nomToLFMap;

	public NewIntentionRecognizer() {
		super();
		nomToLFMap = new HashMap<String, SelectedLogicalForm>();
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
		solvers.addSolver(new ReferenceResolutionAssertionSolver());

		ProofInterpreter<InterpretedUserIntention> interpreter = new InterpretedUserIntentionProofInterpreter(	);

		final WorkingMemoryWriterComponent committer = this;

		return new AbstractProofInterpretationContext<InterpretedUserIntention>(pruner, expander, solvers, interpreter) {

			@Override
			public void onSuccessfulInterpretation(InterpretedUserIntention i) {
				try {
					getLogger().debug("going to commit the interpretation");
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
	}

	public class ExpandLFAssertionSolver extends AbstractAssertionSolver<String> {

		@Override
		public String parseFromModalisedAtom(ModalisedAtom matom) {
			if (matom.a.predSym.contains("from_logical_form") && matom.a.args.size() == 3
					&& matom.a.args.get(2) instanceof FunctionTerm) {

				String nominal = ((FunctionTerm) matom.a.args.get(2)).functor;
				return nominal;
			}
			return null;
		}

		@Override
		public ContextUpdate solveFromParsed(String a) {
			getLogger().info("solving an LF assertion for nominal \"" + a + "\"");
			final LogicalForm lf = nomToLFMap.get(a).lform;
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

	public class ReferenceResolutionAssertionSolver extends AbstractAssertionSolver<AssertedReferenceAtom> {

		@Override
		public AssertedReferenceAtom parseFromModalisedAtom(ModalisedAtom matom) {
			return AssertedReferenceAtom.fromModalisedAtom(matom);
		}

		@Override
		public ContextUpdate solveFromParsed(AssertedReferenceAtom a) {
			throw new UnsupportedOperationException("Not supported yet.");
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

}
