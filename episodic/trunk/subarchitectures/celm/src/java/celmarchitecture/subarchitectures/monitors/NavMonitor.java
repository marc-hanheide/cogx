package celmarchitecture.subarchitectures.monitors;

import cast.architecture.abstr.ChangeFilterFactory;
import cast.architecture.abstr.WorkingMemoryChangeReceiver;
import cast.architecture.subarchitecture.PrivilegedManagedProcess;
import cast.architecture.subarchitecture.SubarchitectureProcessException;
import cast.cdl.WorkingMemoryAddress;
import cast.cdl.WorkingMemoryChange;
import cast.cdl.WorkingMemoryOperation;
import cast.core.data.CASTData;

import java.util.Date;
import java.text.SimpleDateFormat;

import NavData.RobotPose;
import NavData.TopologicalRobotPos;
import NavData.Person;
import NavData.PersonFollowed;
import NavData.Indication;
import NavData.Goal;

import elm.event.EventSpecificBinaryDataIO;
import elm.event.EventSpecificFeatures;
import celm.conversion.BALTTimeConverter;
import celmarchitecture.global.EventTypeNames;
import celmarchitecture.global.GlobalSettings;

/**
 * NavMonitor is a simple monitor process for changes on Nav WM which might be
 * interesting.
 * 
 * @author Dennis Stachowicz
 */
public class NavMonitor extends SimpleAbstractWMMonitor {

	private TopologicalRobotPos lastTopologicalRobotPos = null;

	public NavMonitor(String _id) {
		super(_id);
	}

	@Override
	public void start() {
		super.start();

		try {
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

			/*
			 * addChangeFilter(ChangeFilterFactory.
			 * createGlobalTypeFilter(RobotPose.class,
			 * WorkingMemoryOperation.ADD), wmcrProcessAddOverwriteEvent);
			 * addChangeFilter(ChangeFilterFactory.
			 * createGlobalTypeFilter(RobotPose.class,
			 * WorkingMemoryOperation.OVERWRITE), wmcrProcessAddOverwriteEvent);
			 */

			addGlobalAddOverwriteFilter(RobotPose.class,
					wmcrProcessAddOverwriteEvent);
			addGlobalAddOverwriteFilter(TopologicalRobotPos.class,
					wmcrProcessAddOverwriteEvent);
			addGlobalAddOverwriteFilter(Person.class,
					wmcrProcessAddOverwriteEvent);
			addGlobalAddOverwriteFilter(PersonFollowed.class,
					wmcrProcessAddOverwriteEvent);
			addGlobalAddOverwriteFilter(Indication.class,
					wmcrProcessAddOverwriteEvent);

			addGlobalAddOverwriteFilter(Goal.class,
					wmcrProcessAddOverwriteEvent);
			addGlobalDeleteFilter(Goal.class, wmcrProcessDeleteEvent);
		} catch (SubarchitectureProcessException e) {
			e.printStackTrace();
			if (GlobalSettings.exitOnException)
				System.exit(GlobalSettings.exitValueOnException);
		}

	}

	protected void processDeleteEvent(WorkingMemoryChange _wmc) {

		try {
			CASTData<?> wme = getWorkingMemoryEntry(_wmc.m_address);

			Object data = wme.getData();

			if (data instanceof Goal) {
				Goal g = (Goal) data;
				EventSpecificFeatures esf = new EventSpecificFeatures(2);
				esf.addKeyValuePair("m_x", "" + g.m_x);
				esf.addKeyValuePair("m_y", "" + g.m_y);
				addPartialEvent("goal deleted", null, null, null, null, esf);
			}

		} catch (SubarchitectureProcessException e) {
			e.printStackTrace();
			if (GlobalSettings.exitOnException)
				System.exit(GlobalSettings.exitValueOnException);
		}
	}

	protected void processAddOverwriteEvent(WorkingMemoryChange _wmc) {

		try {
			CASTData<?> wme = getWorkingMemoryEntry(_wmc.m_address);

			Object data = wme.getData();

			SimpleDateFormat df = new SimpleDateFormat(
					"yyyy-MM-dd HH:mm:ss.SSS");

			if (data instanceof RobotPose) {
				RobotPose rp = (RobotPose) data;
				Date time = BALTTimeConverter.toJavaDate(rp.m_time);
				EventSpecificFeatures esf = new EventSpecificFeatures(4);

				// for testing, debugging and profiling purposes:
				esf.addKeyValuePair("event_reception_time", ""
						+ df.format(new Date()));
				// esf.addKeyValuePair("included_BALTTime", rp.m_time.m_s + ", "
				// + rp.m_time.m_us);
				// esf.addKeyValuePair("included_BALTTime_in_ms", "" +
				// BALTTimeConverter.toMilliseconds(rp.m_time));
				// balt.corba.autogen.FrameworkBasics.BALTTime bt =
				// balt.management.ProcessLauncher.getBALTTime();
				// esf.addKeyValuePair("current_BALTTime", bt.m_s + ", " +
				// bt.m_us);

				esf.addKeyValuePair("m_x", "" + rp.m_x);
				esf.addKeyValuePair("m_y", "" + rp.m_y);
				esf.addKeyValuePair("m_theta", "" + rp.m_theta);
				// esf.addKeyValuePair("m_cov", "" + rp.m_cov);

				addPartialEvent("RobotPose", EventSpecificBinaryDataIO
						.objectToByteArray(rp), time, time, null, esf);
			} else if (data instanceof TopologicalRobotPos) {

				TopologicalRobotPos trp = (TopologicalRobotPos) data;

				if (lastTopologicalRobotPos != null
						&& trp.m_areaID != lastTopologicalRobotPos.m_areaID) {

					EventSpecificFeatures esf = new EventSpecificFeatures(4);

					// for testing, debugging and profiling purposes:
					esf.addKeyValuePair("event_reception_time", ""
							+ df.format(new Date()));

					esf.addKeyValuePair("m_areaID", "" + trp.m_areaID);
					esf.addKeyValuePair("old room", ""
							+ lastTopologicalRobotPos.m_areaID);
					esf.addKeyValuePair("new room", "" + trp.m_areaID);
					addPartialEvent(EventTypeNames.leaveRoom,
							EventSpecificBinaryDataIO.objectToByteArray(trp),
							null, null, null, esf);
					addPartialEvent(EventTypeNames.enterRoom,
							EventSpecificBinaryDataIO.objectToByteArray(trp),
							null, null, null, esf);
				} else {
					EventSpecificFeatures esf = new EventSpecificFeatures(2);

					// for testing, debugging and profiling purposes:
					esf.addKeyValuePair("event_reception_time", ""
							+ df.format(new Date()));

					esf.addKeyValuePair("m_areaID", "" + trp.m_areaID);

					addPartialEvent("TopologicalRobotPos",
							EventSpecificBinaryDataIO.objectToByteArray(trp),
							null, null, null, esf);
				}
				lastTopologicalRobotPos = trp;
			} else if (data instanceof PersonFollowed) {
				PersonFollowed pf = (PersonFollowed) data;
				if (pf.m_id != -1) {
					Date time = BALTTimeConverter.toJavaDate(pf.m_time);
					EventSpecificFeatures esf = new EventSpecificFeatures(2);
					// for testing, debugging and profiling purposes:
					esf.addKeyValuePair("event_reception_time", ""
							+ df.format(new Date()));
					esf.addKeyValuePair("m_id", "" + pf.m_id);
					addPartialEvent("PersonFollowed", EventSpecificBinaryDataIO
							.objectToByteArray(pf), time, time, null, esf);
				}
				// else: no person followed, ignore
			} else if (data instanceof Person) {
				Person person = (Person) data;

				Date time = BALTTimeConverter.toJavaDate(person.m_time);
				EventSpecificFeatures esf = new EventSpecificFeatures(8);
				// for testing, debugging and profiling purposes:
				esf.addKeyValuePair("event_reception_time", ""
						+ df.format(new Date()));
				esf.addKeyValuePair("m_id", "" + person.m_id);
				esf.addKeyValuePair("m_x", "" + person.m_x);
				esf.addKeyValuePair("m_y", "" + person.m_y);
				esf.addKeyValuePair("m_theta", "" + person.m_theta);
				esf.addKeyValuePair("m_speed", "" + person.m_speed);
				esf.addKeyValuePair("m_visibility", "" + person.m_visibility);
				esf.addKeyValuePair("m_areaID", "" + person.m_areaID);

				addPartialEvent("Person", EventSpecificBinaryDataIO
						.objectToByteArray(person), time, time, null, esf);

				// else: no person followed, ignore
			} else if (data instanceof Indication) {
				Indication ind = (Indication) data;
				EventSpecificFeatures esf = new EventSpecificFeatures(1);
				esf.addKeyValuePair("m_object", ind.m_object);
				addPartialEvent("Indication", EventSpecificBinaryDataIO
						.objectToByteArray(ind), null, null, null, esf);
			} else if (data instanceof Goal) {
				Goal g = (Goal) data;
				EventSpecificFeatures esf = new EventSpecificFeatures(2);
				esf.addKeyValuePair("m_x", "" + g.m_x);
				esf.addKeyValuePair("m_y", "" + g.m_y);
				addPartialEvent("Goal", EventSpecificBinaryDataIO
						.objectToByteArray(g), null, null, null, esf);
			}

		} catch (java.io.IOException e) {
			e.printStackTrace();
			if (GlobalSettings.exitOnException)
				System.exit(GlobalSettings.exitValueOnException);
		} catch (SubarchitectureProcessException e) {
			e.printStackTrace();
			if (GlobalSettings.exitOnException)
				System.exit(GlobalSettings.exitValueOnException);
		}
	}

}