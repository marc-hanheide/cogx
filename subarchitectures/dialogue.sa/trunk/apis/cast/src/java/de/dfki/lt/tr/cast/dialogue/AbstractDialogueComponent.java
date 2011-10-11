package de.dfki.lt.tr.cast.dialogue;

import cast.architecture.ManagedComponent;
import cast.cdl.WorkingMemoryAddress;
import cast.core.CASTComponent;

import java.util.Map;
import java.util.concurrent.BlockingQueue;
import java.util.concurrent.LinkedBlockingQueue;

public abstract class AbstractDialogueComponent extends ManagedComponent {

	private boolean running;
	private final BlockingQueue<ProcessingTask> taskQueue;

	private final ProcessingTask EAT_POISON_PILL;

	public AbstractDialogueComponent() {
		taskQueue = new LinkedBlockingQueue<ProcessingTask>();
		running = true;

		EAT_POISON_PILL = new ProcessingTaskWithoutData() {
			@Override
			public void execute() {
				getLogger().debug("swallowing the poison pill");
				running = false;
			}
		};
	}

	@Override
	public final void configure(Map<String, String> args) {
		super.configure(args);
		onConfigure(args);
	}

	protected void onConfigure(Map<String, String> args) {
	}

	@Override
	public final void start() {
		super.start();
		onStart();
	}

	protected void onStart() {
	}

	@Override
	public final void stop() {
		scheduleOwnDeath();
	}

	protected final void addTask(ProcessingTask t) {
		taskQueue.add(t);
	}

	protected final void scheduleOwnDeath() {
		addTask(EAT_POISON_PILL);
	}

	@Override
	protected final void runComponent() {
		try {
			while (running) {
				Runnable r = taskQueue.take();
				r.run();
			}
		} catch (InterruptedException ex) {
			getLogger().warn("interrupted, exiting runComponent", ex);
		}
	}

	public interface ProcessingTask extends Runnable {
	}

	public abstract static class ProcessingTaskWithData<T> implements
			ProcessingTask {

		private final T data;

		public ProcessingTaskWithData(T data) {
			this.data = data;
		}

		public abstract void execute(T data);

		@Override
		public final void run() {
			execute(data);
		}

	}

	public abstract static class ProcessingTaskWithDataAndComponent<T, C extends CASTComponent> extends
			ProcessingTaskWithData<T> {
		private final C m_component;

		public ProcessingTaskWithDataAndComponent(T _data,
				C _component) {
			super(_data);
			m_component = _component;
		}

		public C getComponent() {
			return m_component;
		}
	}

	public abstract static class ProcessingTaskWithoutData implements
			ProcessingTask {

		public abstract void execute();

		@Override
		public final void run() {
			execute();
		}
	}

	public static String wmaToString(WorkingMemoryAddress wma) {
		if (wma != null) {
			return "[" + wma.id + "," + wma.subarchitecture + "]";
		}
		else {
			return "NULL";
		}
	}

}
