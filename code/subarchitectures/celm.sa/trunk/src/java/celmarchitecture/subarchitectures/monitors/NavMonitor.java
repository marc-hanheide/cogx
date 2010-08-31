package celmarchitecture.subarchitectures.monitors;

import cast.SubarchitectureComponentException;
import cast.architecture.ChangeFilterFactory;
import cast.architecture.ManagedComponent;
import cast.architecture.WorkingMemoryChangeReceiver;
import cast.cdl.WorkingMemoryChange;
import cast.cdl.WorkingMemoryOperation;
import cast.core.CASTData;

import java.util.Date;
import java.text.SimpleDateFormat;

import NavData.RobotPose2d;
import NavData.TopologicalRobotPos;
import NavData.Person;
import NavData.PersonFollowed;

import elm.event.EventSpecificBinaryDataIO;
import elm.event.EventSpecificFeatures;
import celm.conversion.CASTTimeConverter;
import celmarchitecture.global.EventTypeNames;
import celmarchitecture.global.GlobalSettings;
import celmarchitecture.subarchitectures.abstr.SimpleAbstractWMMonitor;

/**
 * NavMonitor is a simple monitor process for changes on Nav WM which might be
 * interesting. It can serve as an example of the "traditional" (non-plugin) 
 * approach to CELM monitoring.
 * 
 * @author Dennis Stachowicz
 */
public class NavMonitor extends SimpleAbstractWMMonitor {

	private TopologicalRobotPos lastTopologicalRobotPos = null;

	public NavMonitor() {
		super();
	}

	@Override
	public void start() {
		super.start();

		
		WorkingMemoryChangeReceiver wmcrProcessAddOverwriteEvent = new WorkingMemoryChangeReceiver() {

			public void workingMemoryChanged(WorkingMemoryChange _wmc) {
				// log(CASTUtils.toString(_wmc));
				processAddOverwriteEvent(_wmc);
			}
		};
		WorkingMemoryChangeReceiver wmcrProcessDeleteEvent = new WorkingMemoryChangeReceiver() {

			public void workingMemoryChanged(WorkingMemoryChange _wmc) {
				// log(CASTUtils.toString(_wmc));
				processDeleteEvent(_wmc);
			}
		};


		addGlobalAddOverwriteFilter(RobotPose2d.class,
				wmcrProcessAddOverwriteEvent);
		addGlobalAddOverwriteFilter(TopologicalRobotPos.class,
				wmcrProcessAddOverwriteEvent);
		addGlobalAddOverwriteFilter(Person.class,
				wmcrProcessAddOverwriteEvent);
		addGlobalAddOverwriteFilter(PersonFollowed.class,
				wmcrProcessAddOverwriteEvent);

	}

	protected void processDeleteEvent(WorkingMemoryChange _wmc) {

		try {
			CASTData<?> wme = getWorkingMemoryEntry(_wmc.address);

			Object data = wme.getData();

		} catch (SubarchitectureComponentException e) {
			e.printStackTrace();
			if (GlobalSettings.exitOnException)
				System.exit(GlobalSettings.exitValueOnException);
		}
	}

	protected void processAddOverwriteEvent(WorkingMemoryChange _wmc) {

		try {
			CASTData<?> wme = getWorkingMemoryEntry(_wmc.address);

			Object data = wme.getData();

			SimpleDateFormat df = new SimpleDateFormat(
					"yyyy-MM-dd HH:mm:ss.SSS");

			if (data instanceof RobotPose2d) {
				RobotPose2d rp = (RobotPose2d) data;
				Date time = CASTTimeConverter.toJavaDate(rp.time);
				EventSpecificFeatures esf = new EventSpecificFeatures(4);

				// for testing, debugging and profiling purposes:
				esf.addKeyValuePair("event_reception_time", ""
						+ df.format(new Date()));
				// esf.addKeyValuePair("included_CASTTime", rp.time.s + ", "
				// + rp.time.us);
				// esf.addKeyValuePair("included_CASTTime_in_ms", "" +
				// CASTTimeConverter.toMilliseconds(rp.time));
				// balt.corba.autogen.FrameworkBasics.CASTTime bt =
				// balt.management.ProcessLauncher.getCASTTime();
				// esf.addKeyValuePair("current_CASTTime", bt.s + ", " +
				// bt.us);

				esf.addKeyValuePair("x", "" + rp.x);
				esf.addKeyValuePair("y", "" + rp.y);
				esf.addKeyValuePair("theta", "" + rp.theta);
				// esf.addKeyValuePair("cov", "" + rp.cov);

				addPartialEvent("RobotPose2d", null, // EventSpecificBinaryDataIO.objectToByteArray(rp), 
						time, time, esf);
			} else if (data instanceof TopologicalRobotPos) {

				TopologicalRobotPos trp = (TopologicalRobotPos) data;

				if (lastTopologicalRobotPos != null
						&& trp.areaId != lastTopologicalRobotPos.areaId) {

					EventSpecificFeatures esf = new EventSpecificFeatures(4);

					// for testing, debugging and profiling purposes:
					esf.addKeyValuePair("event_reception_time", ""
							+ df.format(new Date()));

					esf.addKeyValuePair("areaId", "" + trp.areaId);
					esf.addKeyValuePair("old room", ""
							+ lastTopologicalRobotPos.areaId);
					esf.addKeyValuePair("new room", "" + trp.areaId);
					addPartialEvent(EventTypeNames.leaveRoom,
							EventSpecificBinaryDataIO.objectToByteArray(trp),
							null, null, esf);
					addPartialEvent(EventTypeNames.enterRoom,
							EventSpecificBinaryDataIO.objectToByteArray(trp),
							null, null, esf);
				} else {
					EventSpecificFeatures esf = new EventSpecificFeatures(2);

					// for testing, debugging and profiling purposes:
					esf.addKeyValuePair("event_reception_time", ""
							+ df.format(new Date()));

					esf.addKeyValuePair("areaId", "" + trp.areaId);

					addPartialEvent("TopologicalRobotPos",
							EventSpecificBinaryDataIO.objectToByteArray(trp),
							null, null, esf);
				}
				lastTopologicalRobotPos = trp;
			} else if (data instanceof PersonFollowed) {
				PersonFollowed pf = (PersonFollowed) data;
				if (pf.id != -1) {
					Date time = CASTTimeConverter.toJavaDate(pf.time);
					EventSpecificFeatures esf = new EventSpecificFeatures(2);
					// for testing, debugging and profiling purposes:
					esf.addKeyValuePair("event_reception_time", ""
							+ df.format(new Date()));
					esf.addKeyValuePair("id", "" + pf.id);
					addPartialEvent("PersonFollowed", EventSpecificBinaryDataIO
							.objectToByteArray(pf), time, time, esf);
				}
				// else: no person followed, ignore
			} else if (data instanceof Person) {
				Person person = (Person) data;

				Date time = CASTTimeConverter.toJavaDate(person.time);
				EventSpecificFeatures esf = new EventSpecificFeatures(7);
				// for testing, debugging and profiling purposes:
				esf.addKeyValuePair("event_reception_time", ""
						+ df.format(new Date()));
				esf.addKeyValuePair("id", "" + person.id);
				esf.addKeyValuePair("x", "" + person.x);
				esf.addKeyValuePair("y", "" + person.y);
				esf.addKeyValuePair("direction", "" + person.direction);
				esf.addKeyValuePair("speed", "" + person.speed);
				// esf.addKeyValuePair("visibility", "" + person.visibility);
				esf.addKeyValuePair("areaId", "" + person.areaId);

				addPartialEvent("Person", EventSpecificBinaryDataIO
						.objectToByteArray(person), time, time, esf);

				// else: no person followed, ignore
			
			} 

		} catch (java.io.IOException e) {
			e.printStackTrace();
			if (GlobalSettings.exitOnException)
				System.exit(GlobalSettings.exitValueOnException);
		} catch (SubarchitectureComponentException e) {
			e.printStackTrace();
			if (GlobalSettings.exitOnException)
				System.exit(GlobalSettings.exitValueOnException);
		}
	}

}
