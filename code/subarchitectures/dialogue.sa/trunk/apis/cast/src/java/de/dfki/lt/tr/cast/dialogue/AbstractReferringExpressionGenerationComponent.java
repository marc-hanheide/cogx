package de.dfki.lt.tr.cast.dialogue;

import cast.SubarchitectureComponentException;
import cast.architecture.ChangeFilterFactory;
import cast.architecture.WorkingMemoryChangeReceiver;
import cast.cdl.WorkingMemoryAddress;
import cast.cdl.WorkingMemoryChange;
import cast.cdl.WorkingMemoryOperation;
import de.dfki.lt.tr.dialogue.production.ProductionUtils;
import de.dfki.lt.tr.dialogue.production.ReferenceGenerationRequest;
import de.dfki.lt.tr.dialogue.production.ReferenceGenerationResult;
import de.dfki.lt.tr.dialogue.production.ReferringExpressionGenerator;

public abstract class AbstractReferringExpressionGenerationComponent<T extends ReferringExpressionGenerator>
extends AbstractDialogueComponent {

	private T generator;

	public AbstractReferringExpressionGenerationComponent() {
		super();
		generator = null;
	}

	@Override
	protected void onStart() {
		super.onStart();

		generator = initGenerator();
		if (generator == null) {
			getLogger().fatal("got a null generator! will die.");
			scheduleOwnDeath();
			return;
		}

		addChangeFilter(ChangeFilterFactory.createGlobalTypeFilter(ReferenceGenerationRequest.class, WorkingMemoryOperation.ADD), 
				new WorkingMemoryChangeReceiver() {
			@Override
			public void workingMemoryChanged(WorkingMemoryChange _wmc) {
				addTask(new ProcessingTaskWithData<WorkingMemoryChange>(_wmc) {

					@Override
					public void execute(WorkingMemoryChange arg) {
						processResolutionRequest(arg);
					}

				});
			};
		});
	}

	/**
	 * Called after configure(...).
	 *
	 * @return the generator
	 */
	protected abstract T initGenerator();

	protected final T getGenerator() {
		return generator;
	}

	private void processResolutionRequest(final WorkingMemoryChange _wmc) {
		getLogger().debug("Got a callback for ReferenceResolutionRequest " +  wmaToString(_wmc.address)+ ": " + _wmc.operation.toString());

		try {
			final ReferenceGenerationRequest rr = this.getMemoryEntry(_wmc.address, ReferenceGenerationRequest.class);
			addTask(new ProcessingTaskWithData<WorkingMemoryChange>(_wmc) {

				@Override
				public void execute(WorkingMemoryChange _wmc) {
					try {
						getLogger().debug("will act on the ReferenceGenerationRequest [" + _wmc.address.id + "," + _wmc.address.subarchitecture + "]");
						ReferenceGenerationResult result = getGenerator().generate(rr, _wmc.address);
						if (result != null) {
							WorkingMemoryAddress wma = new WorkingMemoryAddress(newDataID(), getSubarchitectureID());
							getLogger().debug("adding a GRE result " + wmaToString(wma) + ":\n" + ProductionUtils.referenceGenerationResultToString(result));
							addToWorkingMemory(wma, result);
						}
						else {
							getLogger().warn("unable to generate (resolver returned null)");
						}
					}
					catch (SubarchitectureComponentException ex) {
						getLogger().error("subarch component exception", ex);
					}
				}
			});
		}
		catch (SubarchitectureComponentException ex) {
			getLogger().error("subarch component exception", ex);
		}
	}

}
