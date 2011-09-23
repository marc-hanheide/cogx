package de.dfki.lt.tr.cast.dialogue;

import de.dfki.lt.tr.dialogue.ref.newiface.ReferenceResolver;
import cast.SubarchitectureComponentException;
import cast.architecture.ChangeFilterFactory;
import cast.architecture.WorkingMemoryChangeReceiver;
import cast.cdl.WorkingMemoryChange;
import cast.cdl.WorkingMemoryOperation;
import de.dfki.lt.tr.dialogue.interpret.ReferenceUtils;
import de.dfki.lt.tr.dialogue.ref.ReferenceResolutionRequest;
import de.dfki.lt.tr.dialogue.ref.ReferenceResolutionResult;

public abstract class AbstractReferenceResolutionComponent<T extends ReferenceResolver>
extends AbstractDialogueComponent {

	private final T resolver;

	public AbstractReferenceResolutionComponent(T resolver) {
		super();
		this.resolver = resolver;
	}

	@Override
	protected void onStart() {
		addChangeFilter(ChangeFilterFactory.createGlobalTypeFilter(ReferenceResolutionRequest.class, WorkingMemoryOperation.ADD), 
				new WorkingMemoryChangeReceiver() {
					@Override
					public void workingMemoryChanged(WorkingMemoryChange _wmc) {
						processResolutionRequest(_wmc);
					};
		});
	}

	protected final T getResolver() {
		return resolver;
	}

	private void processResolutionRequest(final WorkingMemoryChange _wmc) {
		getLogger().debug("Got a callback for ReferenceResolutionRequest [" + _wmc.address.id + "," + _wmc.address.subarchitecture + "]: " + _wmc.operation.toString());

		try {
			final ReferenceResolutionRequest rr = this.getMemoryEntry(_wmc.address, ReferenceResolutionRequest.class);
			addTask(new ProcessingTaskWithData<WorkingMemoryChange>(_wmc) {

				@Override
				public void execute(WorkingMemoryChange _wmc) {
					try {
						getLogger().debug("will act on the ReferenceResolutionRequest [" + _wmc.address.id + "," + _wmc.address.subarchitecture + "]");
						ReferenceResolutionResult res = resolver.resolve(rr);
						if (res != null) {
							getLogger().debug("overwriting the request " + wmaToString(_wmc.address) + " with result:\n" + ReferenceUtils.resolutionResultToString(res));
							overwriteWorkingMemory(_wmc.address, res);
						}
						else {
							getLogger().warn("unable to resolve the reference (got null)");
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
