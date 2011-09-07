package de.dfki.lt.tr.cast.dialogue;

import cast.SubarchitectureComponentException;
import java.util.Map;

import cast.architecture.ChangeFilterFactory;
import cast.architecture.ManagedComponent;
import cast.architecture.WorkingMemoryChangeReceiver;
import cast.cdl.WorkingMemoryAddress;
import cast.cdl.WorkingMemoryChange;
import cast.cdl.WorkingMemoryOperation;
import de.dfki.lt.tr.dialogue.interpret.ReferenceUtils;
import de.dfki.lt.tr.dialogue.ref.ResolutionRequest;
import de.dfki.lt.tr.dialogue.ref.ResolutionResult;
import java.util.concurrent.BlockingQueue;
import java.util.concurrent.LinkedBlockingQueue;

public abstract class AbstractReferenceResolutionComponent
extends ManagedComponent
implements ReferenceResolver {

	private boolean running;
	private final BlockingQueue<Runnable> taskQueue;

	public AbstractReferenceResolutionComponent() {
		taskQueue = new LinkedBlockingQueue<Runnable>();
		running = true;
	}

	@Override
	public void configure(Map<String, String> args) {
		super.configure(args);
	}

	@Override
	public void start() {
		addChangeFilter(ChangeFilterFactory.createGlobalTypeFilter(ResolutionRequest.class, WorkingMemoryOperation.ADD), 
				new WorkingMemoryChangeReceiver() {
					@Override
					public void workingMemoryChanged(WorkingMemoryChange _wmc) {
						processResolutionRequest(_wmc);
					};
		});
		onStart();
	}

	protected abstract void onStart();

	@Override
	public void stop() {
		taskQueue.add(new Runnable() {
			@Override
			public void run() {
				running = false;
			}
		});
	}

	private void processResolutionRequest(final WorkingMemoryChange _wmc) {
		getLogger().debug("Got a callback for ResolutionRequest [" + _wmc.address.id + "," + _wmc.address.subarchitecture + "]: " + _wmc.operation.toString());

		try {
			final ResolutionRequest rr = this.getMemoryEntry(_wmc.address, ResolutionRequest.class);
			taskQueue.add(new Runnable() {

				@Override
				public void run() {
					try {
						getLogger().debug("will act on the ResolutionRequest [" + _wmc.address.id + "," + _wmc.address.subarchitecture + "]");
						ResolutionResult res = resolve(rr);
						if (res != null) {
							getLogger().debug("overwriting the request [" + _wmc.address.id + "," + _wmc.address.subarchitecture + "] with result:\n" + ReferenceUtils.resolutionResultToString(res));
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

	@Override
	public abstract ResolutionResult resolve(ResolutionRequest rr);

	@Override
	protected void runComponent() {
		try {
			while (running) {
				Runnable r = taskQueue.take();
				r.run();
			}
		}
		catch (InterruptedException ex) {
			getLogger().warn("interrupted", ex);
		}
	}

}
