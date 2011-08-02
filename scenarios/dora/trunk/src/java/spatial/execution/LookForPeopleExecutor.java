package spatial.execution;

import java.util.Collections;
import java.util.HashSet;
import java.util.Set;

import VisionData.PeopleDetectionCommand;
import VisionData.Person;
import cast.AlreadyExistsOnWMException;
import cast.CASTException;
import cast.architecture.ChangeFilterFactory;
import cast.architecture.ManagedComponent;
import cast.architecture.WorkingMemoryChangeReceiver;
import cast.cdl.WorkingMemoryChange;
import cast.cdl.WorkingMemoryOperation;
import execution.slice.actions.LookForPeople;
import execution.slice.person.PersonObservation;

/**
 * Executor which
 * 
 * @author nah
 * 
 */
public class LookForPeopleExecutor extends PanAndLookExecutor<LookForPeople> {

	Set<Person> observations = Collections
			.synchronizedSet(new HashSet<Person>());

	class ObservationCollector implements WorkingMemoryChangeReceiver {

		@Override
		public void workingMemoryChanged(WorkingMemoryChange arg0)
				throws CASTException {
			Person p = getComponent()
					.getMemoryEntry(arg0.address, Person.class);
			synchronized (observations) {
				observations.add(p);
			}
		}

	}

	public LookForPeopleExecutor(ManagedComponent _component, int _detections) {
		super(_component, LookForPeople.class, _detections);
	}

	@Override
	protected void triggerDetection() {
		getComponent().log("detection triggered");

		// Fire off a detection command
		PeopleDetectionCommand detect = new PeopleDetectionCommand();
		String id = getComponent().newDataID();
		try {
			getComponent().addChangeFilter(
					ChangeFilterFactory.createGlobalTypeFilter(Person.class,
							WorkingMemoryOperation.ADD),
					new ObservationCollector());
			getComponent().addChangeFilter(
					ChangeFilterFactory.createIDFilter(id,
							WorkingMemoryOperation.DELETE),
					getAfterDetectionReceiver());
			getComponent().addToWorkingMemory(id, detect);

		} catch (AlreadyExistsOnWMException e) {
			e.printStackTrace();
		}
	}

	@Override
	protected boolean acceptAction(LookForPeople _action) {
		return true;
	}

	@Override
	protected void publishActionOutcome() {
		PersonObservation po = new PersonObservation(0.0);
		synchronized (observations) {
			for (Person p : observations) {
				po.existProb += p.existProb;
			}
			po.existProb /= observations.size();
			try {
				getComponent().addToWorkingMemory(getComponent().newDataID(),
						po);
			} catch (AlreadyExistsOnWMException e) {
				getComponent().logException(e);
			}
			observations.clear();
		}
	}

}
