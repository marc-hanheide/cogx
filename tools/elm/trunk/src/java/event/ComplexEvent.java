/** _
 *  part of the ELM system.
 *  @author Dennis Stachowicz
 */

package elm.event;

import java.util.Vector;
import java.util.Collection;
import java.util.Iterator;
import java.util.NoSuchElementException;

public class ComplexEvent extends Event {

	/**
	 * minimum constructor for composite events
	 * 
	 * @throws java.util.NoSuchElementException
	 *             if subEvents is empty!
	 * @throws EventIDException
	 *             if called with a subEvent which has no eventID assigned!
	 */
	public ComplexEvent(EventType eventType, Collection<Event> subEvents)
			throws NoSuchElementException, EventIDException {
		this(eventType, subEvents, null);
	}

	/**
	 * minimum constructor for composite events
	 * 
	 * @throws java.util.NoSuchElementException
	 *             if subEvents is empty!
	 * @throws EventIDException
	 *             if called with a subEvent which has no eventID assigned!
	 */
	public ComplexEvent(EventType eventType, Collection<Event> subEvents,
			EventSpecificFeatures esf) throws NoSuchElementException,
			EventIDException {
		super(null, true, degreeUndefined, eventType, null, null,
				(byte[]) null, null, null, esf);

		int sDegrees = degreeUndefined;
		Event e = null;
		Iterator<Event> iterator = subEvents.iterator();

		Vector<EventID> subEventIDs = new Vector<EventID>(subEvents.size());

		while (iterator.hasNext()) {
			e = iterator.next();
			if (!e.hasEventIDAssigned())
				throw new EventIDException("all subEvents must have IDs "
						+ "assigned already to form a composite event!");

			subEventIDs.add(e.getEventID());
			sDegrees = (sDegrees > e.getDegree() ? sDegrees : e.getDegree());
		}

		EventTime time = new EventTime(subEvents);

		// take the first subEvent's location and have its EventLocationFactory
		// produce
		// the new location from all the subEvent's locations
		// (throws java.util.NoSuchElementException if subEvents is empty!)
		EventLocation location = subEvents.iterator().next().getLocation()
				.getFactory().fromSubEvents(subEvents);

		// this.physicalEntityIDs = // still have to be fixed!!!!

		init(true, sDegrees + 1, eventType, time, location, null, subEventIDs,
				null, esf);
	}

	/**
	 * minimum constructor for composite events
	 * 
	 * @throws java.util.NoSuchElementException
	 *             if subEvents is empty!
	 * @throws EventIDException
	 *             if called with a subEvent which has no eventID assigned!
	 */
	public ComplexEvent(EventType eventType, Event[] subEvents)
			throws NoSuchElementException, EventIDException {
		this(eventType, subEvents, null);
	}

	/**
	 * minimum constructor for composite events
	 * 
	 * @throws java.util.NoSuchElementException
	 *             if subEvents is empty!
	 * @throws EventIDException
	 *             if called with a subEvent which has no eventID assigned!
	 */
	public ComplexEvent(EventType eventType, Event[] subEvents,
			EventSpecificFeatures esf) throws NoSuchElementException,
			EventIDException {
		super(null, true, degreeUndefined, eventType, null, null,
				(byte[]) null, null, null, esf);

		int sDegrees = degreeUndefined;

		Vector<EventID> subEventIDs = new Vector<EventID>(subEvents.length);

		for (int i = 0; i < subEvents.length; i++) {

			if (!subEvents[i].hasEventIDAssigned())
				throw new EventIDException("all subEvents must have IDs "
						+ "assigned already to form a composite event!");

			subEventIDs.add(subEvents[i].getEventID());

			sDegrees = (sDegrees > subEvents[i].getDegree() ? sDegrees
					: subEvents[i].getDegree());
		}

		EventTime time = new EventTime(subEvents);

		// take the first subEvent's location and have its EventLocationFactory
		// produce
		// the new location from all the subEvent's locations
		// (throw java.util.NoSuchElementException if subEvents is empty!)
		if (subEvents.length < 1)
			throw new NoSuchElementException(
					"subEvents array must not be empty!");
		EventLocation location = subEvents[0].getLocation().getFactory()
				.fromSubEvents(subEvents);

		// this.physicalEntityIDs = // still have to be fixed!!!!

		init(true, sDegrees + 1, eventType, time, location, null, subEventIDs,
				null, esf);
	}
}
