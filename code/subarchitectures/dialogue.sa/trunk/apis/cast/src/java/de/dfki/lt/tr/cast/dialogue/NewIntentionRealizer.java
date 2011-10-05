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
import de.dfki.lt.tr.dialogue.interpret.ConversionUtils;
import de.dfki.lt.tr.dialogue.interpret.IntentionManagementConstants;
import de.dfki.lt.tr.dialogue.interpret.RobotsIntentionToAct;
import de.dfki.lt.tr.dialogue.interpret.RobotsIntentionToActProofInterpreter;
import de.dfki.lt.tr.dialogue.slice.interpret.InterpretationRequest;
import de.dfki.lt.tr.infer.abducer.engine.AbductionEnginePrx;
import de.dfki.lt.tr.infer.abducer.engine.FileReadErrorException;
import de.dfki.lt.tr.infer.abducer.engine.SyntaxErrorException;
import de.dfki.lt.tr.infer.abducer.lang.ModalisedAtom;
import de.dfki.lt.tr.infer.abducer.lang.Modality;
import de.dfki.lt.tr.infer.abducer.lang.Term;
import de.dfki.lt.tr.infer.abducer.proof.AbstractProofInterpretationContext;
import de.dfki.lt.tr.infer.abducer.proof.AssertionSolverCascade;
import de.dfki.lt.tr.infer.abducer.proof.EngineProofExpander;
import de.dfki.lt.tr.infer.abducer.proof.ProofExpander;
import de.dfki.lt.tr.infer.abducer.proof.ProofInterpretationContext;
import de.dfki.lt.tr.infer.abducer.proof.ProofInterpreter;
import de.dfki.lt.tr.infer.abducer.proof.ProofPruner;
import de.dfki.lt.tr.infer.abducer.proof.pruners.LengthPruner;
import de.dfki.lt.tr.infer.abducer.util.AbductionEngineConnection;
import de.dfki.lt.tr.infer.abducer.util.TermAtomFactory;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;

public class NewIntentionRealizer
extends AbstractAbductiveComponent<RobotsIntentionToAct> {

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

	private int timeout = DEFAULT_TIMEOUT;

	public NewIntentionRealizer() {
		super();
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
	protected ProofInterpretationContext<RobotsIntentionToAct> initContext() {
		ProofPruner pruner = new LengthPruner(3);
		ProofExpander expander = new EngineProofExpander(getEngine(), timeout);

		AssertionSolverCascade solvers = new AssertionSolverCascade();
//		solvers.addSolver(new ExpandLFAssertionSolver());
//		solvers.addSolver(new IntentionIDAssertionSolver());
//		solvers.addSolver(new NewBeliefAssertionSolver());
//		solvers.addSolver(new ReferenceResolutionAssertionSolver());

		ProofInterpreter<RobotsIntentionToAct> interpreter = new RobotsIntentionToActProofInterpreter(getLogger());

		final WorkingMemoryWriterComponent committer = this;

		return new AbstractProofInterpretationContext<RobotsIntentionToAct>(pruner, expander, solvers, interpreter) {

			@Override
			public void onSuccessfulInterpretation(RobotsIntentionToAct i) {
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
//									nomToLFMap.put(slf.lform.root.nomVar, slf);
									InterpretationRequest inprRequest = new InterpretationRequest(intentionToActToGoal(actint, addr));
									addNewPartialInterpretation(addr, interpretationRequestToPartialInterpretation(getContext().getPruner(), addr, inprRequest));
								}
								catch (SubarchitectureComponentException ex) {
									logException(ex);
								}
							}
							
						});
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

	
}
