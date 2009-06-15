package nav.elm;

import java.io.IOException;
import java.util.ArrayList;
import java.util.NoSuchElementException;

import NavData.TopologicalRobotPos;
import cast.core.CASTUtils;
import celmarchitecture.subarchitectures.recognition.Recognizer;
import elm.event.ComplexEvent;
import elm.event.Event;
import elm.event.EventIDException;
import elm.event.EventSpecificFeatures;
import elm.event.EventType;
import elm.eventrecognition.SuperEventRecognizer;

/**
 * Produces "InArea" events where this is the time spent in a particular
 * topological area. The assumption is that the robot is always in an area.
 * 
 * TODO Handle area relabelling
 * 
 * @author nah
 * 
 */
public class NavEventRecognizer extends Recognizer implements
		SuperEventRecognizer {

	private long m_currentAreaID;
	private Event m_currentAreaEntryEvent;

	public NavEventRecognizer() {
		registerRecognizer(this);
		m_currentAreaID = Long.MIN_VALUE;
	}

	public void newEvent(Event _e) {
		// look for topological position changes
		if (_e.getEventType().getName().equals(
				CASTUtils.typeName(TopologicalRobotPos.class))) {
			topologicalPositionEvent(_e);
		}
	}

	/**
	 * @param _e
	 */
	private void topologicalPositionEvent(Event _e) {
		assert (_e.getEventType().getName().equals(CASTUtils
				.typeName(TopologicalRobotPos.class)));

		// Test the reading from binary. This could (should? -- keeps it
		// independent of actual modal data) also be accomplished
		// using the EventSpecificFeatures
		try {
			TopologicalRobotPos trp = (TopologicalRobotPos) _e
					.getEventSpecificBinaryDataAsRawObject();
			long newAreaID = trp.areaId;

			// start-up condition
			if (m_currentAreaEntryEvent == null) {
				m_currentAreaID = newAreaID;
				m_currentAreaEntryEvent = _e;
			}
			// change of area data but no area change.. why?
			else if (m_currentAreaID == newAreaID) {
				// no=op
			}
			// area change
			else {
				// Make complex event from current data
				generateInAreaEvent(m_currentAreaID, m_currentAreaEntryEvent,
						_e);

				// Store new data as current area
				m_currentAreaID = newAreaID;
				m_currentAreaEntryEvent = _e;
			}

		} catch (ClassCastException e) {
			e.printStackTrace();
		} catch (IOException e) {
			e.printStackTrace();
		} catch (ClassNotFoundException e) {
			e.printStackTrace();
		} catch (NoSuchElementException e) {
			e.printStackTrace();
		} catch (EventIDException e) {
			e.printStackTrace();
		}
	}

	/**
	 * @param _areaID
	 * @param _entryEvent
	 * @param _exitEvent
	 * @throws EventIDException
	 */
	private void generateInAreaEvent(long _areaID, Event _entryEvent,
			Event _exitEvent) throws EventIDException {
		// Meta data
		EventSpecificFeatures esf = new EventSpecificFeatures();
		esf.addKeyValuePair("areaID", _areaID);

		// Sub-events
		ArrayList<Event> subEvents = new ArrayList<Event>(2);
		// entry event
		subEvents.add(_entryEvent);
		// exit event
		subEvents.add(_exitEvent);

		ComplexEvent inAreaEvent = new ComplexEvent(new EventType(
				NavData.elm.INAREAEVENT.value), subEvents);
		newEventDetected(inAreaEvent);

		log("detected an InArea event: " + _areaID);
	}

}
