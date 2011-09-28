package spatial.execution;

import java.util.ArrayList;
import java.util.Collections;
import java.util.HashSet;
import java.util.Set;

import NavData.RobotPose2d;
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
import facades.SpatialFacade;

/**
 * Executor which
 * 
 * @author nah
 * 
 */
public class LookForPeopleExecutor extends PanAndLookExecutor<LookForPeople> {

	Set<Person> observations = Collections
			.synchronizedSet(new HashSet<Person>());
	private SpatialFacade spatialFacade;

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
		try {
			spatialFacade = SpatialFacade.get(getComponent());
		} catch (CASTException e) {
			logException(e);
			throw new RuntimeException(e.message);
		}
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
		PersonObservation po = new PersonObservation(new ArrayList<Person>(),
				0.0, -1, 0.0, 0.0, 0.0, 0.0);
		synchronized (observations) {
			try {
				RobotPose2d pose = spatialFacade.getPose();
				po.robotX = pose.x;
				po.robotY = pose.y;
				po.robotTheta = pose.theta;
				po.placeId = spatialFacade.getPlace().id;
				
				for (Person p : observations) {
					po.existProb += p.existProb;
					po.persons.add(p);
				}
				
				po.existProb /= observations.size();
				getComponent().addToWorkingMemory(getComponent().newDataID(),
						po);
				observations.clear();
			} catch (CASTException e1) {
				logException(e1);
			} catch (InterruptedException e) {
				logException(e);
			}
		}
	}

}
