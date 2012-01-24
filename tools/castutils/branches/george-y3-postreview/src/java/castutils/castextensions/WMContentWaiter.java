package castutils.castextensions;

import java.util.HashSet;
import java.util.Map;
import java.util.Map.Entry;
import java.util.concurrent.LinkedBlockingQueue;
import java.util.concurrent.TimeUnit;

import org.apache.log4j.Logger;

import cast.CASTException;
import cast.cdl.WorkingMemoryAddress;
import cast.cdl.WorkingMemoryChange;
import castutils.castextensions.WMView.ChangeHandler;

/**
 * The WMContentWaiter is a helper class that tries to find certain matching
 * content in a WMView. It reads this content if it's available or waits until
 * it appears. To decide what content to look or wait for, read() requires a
 * matching function with a simple interface.
 * 
 * @author marc
 * 
 * @param <T>
 *            the type of data to look for in the WMView and working memory
 *            respectively.
 */
public class WMContentWaiter<T extends Ice.ObjectImpl> implements
		ChangeHandler<T> {

    static final int BELIEF_TIMEOUT = 2000;

	/**
	 * an interface realizing a matching function used for the
	 * {@link WMContentWaiter} to match the content in a view with the content
	 * the {@link WMContentWaiter} is waiting for.
	 * 
	 * @author marc
	 * 
	 * @param <ContentType>
	 *            type of the content to be matched
	 */
	public interface ContentMatchingFunction<ContentType extends Ice.ObjectImpl> {
		/**
		 * @param viewContent
		 * @return true if content matches
		 */
		public boolean matches(ContentType viewContent);
	}

	private LinkedBlockingQueue<Entry<WorkingMemoryAddress, T>> eventQueue;
	private WMView<T> view;
	private Logger logger;

	/**
	 * constructor
	 * 
	 * @param view
	 *            the view in which we are waiting for the value to appear.
	 */
	public WMContentWaiter(WMView<T> view) {
		super();
		this.view = view;
		this.logger = Logger.getLogger(WMContentWaiter.class);
		this.eventQueue = new LinkedBlockingQueue<Entry<WorkingMemoryAddress, T>>();
	}

	/**
	 * this implements a blocking read that wait until the view contains to
	 * object we are looking for according to the match function.
	 * 
	 * @param cmf
	 *            the matching function that decides when the content is present
	 *            that we are looking for.
	 * @return an {@link Entry} of the found content.
	 * @throws InterruptedException
	 */
	public Entry<WorkingMemoryAddress, T> read(ContentMatchingFunction<? super T> cmf)
			throws InterruptedException {

		view.registerHandler(this);
		try {
			HashSet<Entry<WorkingMemoryAddress,T>> copySet=new HashSet<Entry<WorkingMemoryAddress, T>>(view.entrySet()); 
			for (Entry<WorkingMemoryAddress, T> entry : copySet) {
				if (cmf.matches(entry.getValue())) {
					logger.trace("found a matching entry already in the view");
					return entry;
				}
			}
			logger
					.trace("the content we are looking for is not yet in the view... have to wait now.");

            long startTime = System.currentTimeMillis();
			// if we come here, we haven't found the data in the current view so
			// we have to be until it appears
			while (System.currentTimeMillis() - startTime < BELIEF_TIMEOUT) {
				Entry<WorkingMemoryAddress, T> entry = eventQueue.poll(1, TimeUnit.SECONDS);
				if (entry==null)
					continue;
				if (cmf.matches(entry.getValue())) {
					logger
							.trace("found a matching entry being added/overwritten in the view");
					return entry;
				}

			}

            throw new InterruptedException("could not find a corresponding belief in time");

		} finally {
			view.unregisterHandler(this);
		}
	}

	/*
	 * (non-Javadoc)
	 * 
	 * @see
	 * castutils.castextensions.WMView.ChangeHandler#entryChanged(java.util.Map,
	 * cast.cdl.WorkingMemoryChange, Ice.ObjectImpl, Ice.ObjectImpl)
	 */
	@Override
	public void entryChanged(Map<WorkingMemoryAddress, T> map,
			final WorkingMemoryChange wmc, final T newEntry, T oldEntry)
			throws CASTException {
		Entry<WorkingMemoryAddress, T> e = new Entry<WorkingMemoryAddress, T>() {
			WorkingMemoryAddress addr = wmc.address;
			T value = newEntry;

			@Override
			public WorkingMemoryAddress getKey() {
				return addr;
			}

			@Override
			public T getValue() {
				return value;
			}

			@Override
			public T setValue(T value) {
				T oldValue = value;
				this.value = value;
				return oldValue;
			}

			/*
			 * (non-Javadoc)
			 * 
			 * @see java.lang.Object#equals(java.lang.Object)
			 */
			@SuppressWarnings("unchecked")
			@Override
			public boolean equals(Object obj) {
				return value.equals(((Entry<WorkingMemoryAddress, T>) obj)
						.getValue())
						&& addr.equals(((Entry<WorkingMemoryAddress, T>) obj)
								.getKey());
			}

		};

		try {
			eventQueue.put(e);
		} catch (InterruptedException e1) {
			Logger.getLogger(WMContentWaiter.class).error(
					"while putting into event queue: ", e1);
		}

	}

}
