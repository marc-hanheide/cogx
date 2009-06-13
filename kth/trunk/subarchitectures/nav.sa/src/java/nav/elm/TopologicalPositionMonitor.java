/**
 * 
 */
package nav.elm;

import java.io.IOException;
import java.text.SimpleDateFormat;
import java.util.Date;

import NavData.TopologicalRobotPos;
import cast.SubarchitectureComponentException;
import cast.architecture.WorkingMemoryChangeReceiver;
import cast.cdl.WorkingMemoryAddress;
import cast.cdl.WorkingMemoryChange;
import celm.conversion.CASTTimeConverter;
import celmarchitecture.subarchitectures.abstr.SimpleAbstractWMMonitor;
import elm.event.EventSpecificBinaryDataIO;
import elm.event.EventSpecificFeatures;

/**
 * An ELM monitor that remebers the nearest node after each pose update. Just
 * for learning and testing. Doesn't really do anything in a smart way, so don't
 * use for inspiration!
 * 
 * 
 * @author nah
 * 
 */
public class TopologicalPositionMonitor extends SimpleAbstractWMMonitor {

	private SimpleDateFormat m_df;
	private TopologicalRobotPos m_pose;

	public TopologicalPositionMonitor() {
		m_df = new SimpleDateFormat("yyyy-MM-dd HH:mm:ss.SSS");
	}

	@Override
	protected void start() {

		// listen for pose changes
		addLocalAddOverwriteFilter(TopologicalRobotPos.class,
				new WorkingMemoryChangeReceiver() {
					public void workingMemoryChanged(WorkingMemoryChange _wmc) {
						robotPoseUpdated(_wmc.address);
					}
				});

	}

	private void robotPoseUpdated(WorkingMemoryAddress _wma) {
		// this monitor is designed to work locally, so sanity check this
		assert (_wma.subarchitecture.equals(getSubarchitectureID()));

		try {
			// update pose from wm
			m_pose = getMemoryEntry(_wma.id, TopologicalRobotPos.class);
			rememberPose();
		} catch (SubarchitectureComponentException e) {
			e.printStackTrace();
		} catch (IOException e) {
			e.printStackTrace();
		}
	}

	private void rememberPose() throws SubarchitectureComponentException,
			IOException {

		assert (m_pose != null);

		Date time = CASTTimeConverter.toJavaDate(getCASTTime());
		EventSpecificFeatures esf = new EventSpecificFeatures(4);

		log("remebering position: " + m_pose.areaId);

		// for testing, debugging and profiling purposes:
		esf.addKeyValuePair("event_reception_time", ""
				+ m_df.format(new Date()));
		esf.addKeyValuePair("area_id", "" + m_pose.areaId);

		addPartialEvent("TopologicalRobotPose", EventSpecificBinaryDataIO
				.objectToByteArray(m_pose), time, time, null, esf);
	}
}
