package nav.elm;

import NavData.elm.INAREAEVENT;
import cast.AlreadyExistsOnWMException;
import cast.DoesNotExistOnWMException;
import cast.UnknownSubarchitectureException;
import cast.architecture.ChangeFilterFactory;
import cast.architecture.ManagedComponent;
import cast.architecture.WorkingMemoryChangeReceiver;
import cast.cdl.WorkingMemoryChange;
import cast.cdl.WorkingMemoryOperation;
import celm.autogen.CELMCueMatchMode;
import celm.autogen.CELMEventCue;
import celm.autogen.CELMEventQuery;
import celm.autogen.CELMStoredEvent;
import celm.conversion.EventConverter;
import elm.event.Event;
import elm.event.WKTParseException;

public class TestRecollector extends ManagedComponent {

	private final EventConverter m_eventConverter;

	public TestRecollector() {
		m_eventConverter = new EventConverter();
	}

	/**
	 * Create a blank query that is safe to write to WM.
	 * 
	 * @return
	 */
	private CELMEventQuery newQuery() {
		return new CELMEventQuery(0, new CELMEventCue(false, 0, 0, false, null,
				false, CELMCueMatchMode.noMatch, null,
				CELMCueMatchMode.noMatch, null, false, false, false, 0, 0,
				CELMCueMatchMode.noMatch, null, CELMCueMatchMode.noMatch, null,
				CELMCueMatchMode.noMatch, null, CELMCueMatchMode.noMatch,
				false, null, false, null), new CELMStoredEvent[0]);
	}

	@Override
	protected void runComponent() {
		
		sleepComponent(5000);
		println("writing query");
		
		if (isRunning()) {
			// create new query
			CELMEventQuery query = newQuery();
			query.limit = 0;

			// match event types
			query.cue.matchEventType = true;
			query.cue.eventType = INAREAEVENT.value;

			String queryID = newDataID();
			addChangeFilter(ChangeFilterFactory.createIDFilter(queryID,
					WorkingMemoryOperation.OVERWRITE),
					new WorkingMemoryChangeReceiver() {

						public void workingMemoryChanged(
								WorkingMemoryChange _wmc) {
							queryAnswered(_wmc);
						}
					});

			
			try {
				addToWorkingMemory(queryID, query);
			} catch (AlreadyExistsOnWMException e) {
				e.printStackTrace();
			}
		}

	}

	private void queryAnswered(WorkingMemoryChange _wmc) {
		try {
			CELMEventQuery query = getMemoryEntry(_wmc.address,
					CELMEventQuery.class);
			for (CELMStoredEvent storedEvent : query.events) {
				println("---------------------------------------");
				println(m_eventConverter.getEvent(storedEvent));
				println("---------------------------------------");
			}

		} catch (DoesNotExistOnWMException e) {
			e.printStackTrace();
		} catch (UnknownSubarchitectureException e) {
			e.printStackTrace();
		} catch (WKTParseException e) {
			e.printStackTrace();
		}
	}
}
