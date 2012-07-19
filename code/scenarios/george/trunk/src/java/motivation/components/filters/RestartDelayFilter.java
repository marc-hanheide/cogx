package motivation.components.filters;

import java.util.Map;

import motivation.slice.Motive;
import cast.CASTException;
import cast.cdl.WorkingMemoryAddress;
import cast.cdl.WorkingMemoryChange;
import cast.core.CASTUtils;
import castutils.castextensions.WMView.ChangeHandler;

/**
 * 
 * Resets delays when an active motive of a higher drive set is completed.
 * 
 * @author nah
 * 
 */
public class RestartDelayFilter extends DelayFilter implements
		ChangeHandler<Motive> {

	private final DriveHierarchy m_driveHierarchy = GeorgeDriveConfig
			.getGeorgeDriveHierarchy();

	public RestartDelayFilter() {
		super();
	}

	@Override
	public void configure(Map<String, String> arg0) {
		// super is necessary
		super.configure(arg0);
	}

	@Override
	public void start() {
		super.start();
		// this is overkill, but I'm not convinced the completion handler will
		// get called at the correct point in the
		m_component.addMotiveActivationHandler(this);
		m_component.addMotiveCompletionHandler(this);
	}

	/**
	 * Called when a motive has been activated.
	 */
	@Override
	public void entryChanged(Map<WorkingMemoryAddress, Motive> map,
			WorkingMemoryChange wmc, Motive newEntry, Motive oldEntry)
			throws CASTException {

		// get the priority of the drive that was just activated
		int activatedPriority = m_driveHierarchy.getPriority(newEntry
				.getClass());

		if (activatedPriority != DriveHierarchy.UNKNOWN_CLASS_VALUE) {

			// these are all the motives which are currently delayed
			for (WorkingMemoryAddress mtvAddr : m_activeDelays.keySet()) {

				// if the delayed motive is of a lower priority than the
				// activated motive

				Motive mtv = map.get(mtvAddr);
				if (mtv != null) {
					Class<? extends Motive> mtvCls = mtv.getClass();
					if (m_driveHierarchy.getPriority(mtvCls) < activatedPriority) {
						// reset the delay to the original value
						m_activeDelays.put(mtvAddr, System.currentTimeMillis()
								+ m_delayMap.get(mtvCls));
						m_component.log("restarting delay of "
								+ CASTUtils.toString(mtvAddr)
								+ " due to activation of "
								+ newEntry.getClass());
					}
				}

			}

		}

	}

}
