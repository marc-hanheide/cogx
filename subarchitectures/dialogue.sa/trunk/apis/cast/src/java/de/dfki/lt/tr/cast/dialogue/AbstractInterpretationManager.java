package de.dfki.lt.tr.cast.dialogue;

import cast.SubarchitectureComponentException;
import cast.architecture.ChangeFilterFactory;
import cast.architecture.WorkingMemoryChangeReceiver;
import cast.cdl.WorkingMemoryAddress;
import cast.cdl.WorkingMemoryChange;
import cast.cdl.WorkingMemoryOperation;
import de.dfki.lt.tr.dialogue.interpret.PartialInterpretation;
import de.dfki.lt.tr.dialogue.slice.interpret.InterpretationRequest;
import de.dfki.lt.tr.infer.abducer.engine.AbductionEnginePrx;
import de.dfki.lt.tr.infer.abducer.proof.Assertion;
import de.dfki.lt.tr.infer.abducer.proof.ProofInterpretationContext;
import de.dfki.lt.tr.infer.abducer.proof.ProofPruner;
import de.dfki.lt.tr.infer.abducer.proof.ProofSet;
import de.dfki.lt.tr.infer.abducer.proof.ProofSet.ExpansionStepResult;
import de.dfki.lt.tr.infer.abducer.util.PrettyPrint;
import java.util.HashMap;
import java.util.Map;
import java.util.concurrent.Executor;
import java.util.concurrent.Executors;

public abstract class AbstractInterpretationManager<T>
extends AbstractDialogueComponent {

	private final Map<WorkingMemoryAddress, PartialInterpretation> interpretations;
	private ProofInterpretationContext<T> context;
	private final Executor executor;

	private AbductionEnginePrx engine;

	public AbstractInterpretationManager() {
		interpretations = new HashMap<WorkingMemoryAddress, PartialInterpretation>();
		executor = Executors.newSingleThreadExecutor();
	}

	@Override
	public void onConfigure(Map<String, String> args) {
		super.onConfigure(args);
	}

	/**
	 * Called after {@code onConfigure()}.
	 */
	protected abstract AbductionEnginePrx initEngine();

	protected final AbductionEnginePrx getEngine() {
		return engine;
	}

	/**
	 * Called after {@code onConfigure()}.
	 */
	protected abstract ProofInterpretationContext<T> initContext();

	protected final ProofInterpretationContext<T> getContext() {
		return context;
	}

	@Override
	public void onStart() {
		super.onStart();

		engine = initEngine();
		if (engine == null) {
			getLogger().fatal("engine is null");
			scheduleOwnDeath();
			return;
		}

		context = initContext();
		if (context == null) {
			getLogger().fatal("interpretation context is null");
			scheduleOwnDeath();
			return;
		}

		addChangeFilter(ChangeFilterFactory.createLocalTypeFilter(
				InterpretationRequest.class, WorkingMemoryOperation.ADD),
				new WorkingMemoryChangeReceiver() {
					@Override
					public void workingMemoryChanged(WorkingMemoryChange _wmc) {
						addTask(new ProcessingTaskWithData<WorkingMemoryAddress>(_wmc.address) {

							@Override
							public void execute(WorkingMemoryAddress addr) {
								try {
									InterpretationRequest inprRequest = getMemoryEntry(addr, InterpretationRequest.class);
									addNewPartialInterpretation(addr, interpretationRequestToPartialInterpretation(context.getPruner(), addr, inprRequest));
								}
								catch (SubarchitectureComponentException ex) {
									logException(ex);
								}
							}
							
						});
					}
				});

	}

	private void addNewPartialInterpretation(WorkingMemoryAddress origin, PartialInterpretation pinpr) {
		getLogger().info("adding a new partial interpretation");
		interpretations.put(origin, pinpr);
		addTask(expandInterpretationTask(pinpr));
	}

	private ProcessingTask expandInterpretationTask(PartialInterpretation pinpr) {
		return new ProcessingTaskWithData<PartialInterpretation>(pinpr) {

			@Override
			public void execute(PartialInterpretation arg) {
				ProofSet proofs = arg.getProofSet();
				ExpansionStepResult<T> result = proofs.expansionStep(getContext());
				if (result.isFinished()) {
					getLogger().info("the expansion process is finished");
					T value = result.getValue();
					if (value != null) {
						// okay, got a result
						getContext().onSuccessfulInterpretation(value);
					}
					else {
						// no result for this one
						getContext().onNoInterpretation();
					}
				}
				else {
					Assertion a = result.getAssertion();
					getLogger().info("will try to act on an assertion: " + PrettyPrint.modalisedAtomToString(a.getModalisedAtom()));
					actOnAssertion(a, arg);
				}
			}

		};
	}

	private void actOnAssertion(final Assertion a, final PartialInterpretation pinpr) {
		// it might actually be useful to use the future here
		executor.execute(new Runnable() {

			@Override
			public void run() {
				getLogger().debug("running the solver on assertion " + PrettyPrint.modalisedAtomToString(a.getModalisedAtom()));
				if (a.process(getContext().getSolver(), engine)) {
					// okay, it's processed successfully
					getLogger().info("assertion solution found");
				}
				else {
					// the assertion actually failed
					getLogger().info("assertion solution failed");
				}
				addTask(expandInterpretationTask(pinpr));
			}
		});
	}

	private PartialInterpretation interpretationRequestToPartialInterpretation(ProofPruner pruner, WorkingMemoryAddress wma, InterpretationRequest request) {
		return PartialInterpretation.fromModalisedAtom(getLogger(), request.goal, pruner);
	}
}
