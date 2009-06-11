/**
 * 
 */
package nav.elm;

import java.io.IOException;
import java.text.SimpleDateFormat;
import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.Date;

import NavData.FNode;
import NavData.RobotPose2d;
import cast.DoesNotExistOnWMException;
import cast.SubarchitectureComponentException;
import cast.architecture.ChangeFilterFactory;
import cast.architecture.WorkingMemoryChangeReceiver;
import cast.cdl.WorkingMemoryAddress;
import cast.cdl.WorkingMemoryChange;
import cast.cdl.WorkingMemoryOperation;
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
public class NearestNodeMonitor extends SimpleAbstractWMMonitor {

	private final double distance(FNode _node, RobotPose2d _pose) {
		double dx = _node.x - _pose.x;
		double dy = _node.y - _pose.y;
		return Math.sqrt((dx * dx) + (dy * dy));
	}

	/**
	 * Comparator which uses the distance from the robot's current pose to order
	 * nodes.
	 * 
	 * @author nah
	 * 
	 */
	private final class NearestNodeComparator implements Comparator<FNode> {

		public int compare(FNode _n1, FNode _n2) {
			// distance from nodes to robot
			double n1d = distance(_n1, m_pose);
			double n2d = distance(_n2, m_pose);

			if (n1d < n2d) {
				return -1;
			} else if (n1d > n2d) {
				return 1;
			} else {
				return 0;
			}
		}
	}

	private ArrayList<FNode> m_nodes;
	private RobotPose2d m_pose;
	private Comparator<FNode> m_comparator;
	private SimpleDateFormat m_df;

	public NearestNodeMonitor() {
		m_df = new SimpleDateFormat("yyyy-MM-dd HH:mm:ss.SSS");
	}

	@Override
	protected void start() {

		// listen for pose changes
		addLocalAddOverwriteFilter(RobotPose2d.class,
				new WorkingMemoryChangeReceiver() {
					public void workingMemoryChanged(WorkingMemoryChange _wmc) {
						robotPoseUpdated(_wmc.address);
					}
				});

		// also listen for new nodes
		addChangeFilter(ChangeFilterFactory.createLocalTypeFilter(FNode.class,
				WorkingMemoryOperation.ADD), new WorkingMemoryChangeReceiver() {
			public void workingMemoryChanged(WorkingMemoryChange _wmc) {
				newNode(_wmc.address);
			}
		});

	}

	private void newNode(WorkingMemoryAddress _wma) {
		// lazy creation of store
		if (m_nodes == null) {
			m_nodes = new ArrayList<FNode>();
			m_comparator = new NearestNodeComparator();
		}

		try {
			// read node from wm
			FNode node = getMemoryEntry(_wma.id, FNode.class);
			// store
			m_nodes.add(node);
			log("collected node: " + node.nodeId);
		} catch (DoesNotExistOnWMException e) {
			e.printStackTrace();
		}
	}

	private void robotPoseUpdated(WorkingMemoryAddress _wma) {
		// this monitor is designed to work locally, so sanity check this
		assert (_wma.subarchitecture.equals(getSubarchitectureID()));

		// no nodes then we don't care about position
		if (m_nodes == null) {
			return;
		}

		try {
			// update pose from wm
			m_pose = getMemoryEntry(_wma.id, RobotPose2d.class);

			// sort node list using position
			Collections.sort(m_nodes, m_comparator);

			// and annouce which is the closest!
			// log("I am closest to node: " + m_nodes.get(0).nodeId);

			rememberNearestNode(m_nodes.get(0));

		} catch (SubarchitectureComponentException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		} catch (IOException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
	}

	private void rememberNearestNode(FNode _node)
			throws SubarchitectureComponentException, IOException {
		Date time = CASTTimeConverter.toJavaDate(getCASTTime());
		EventSpecificFeatures esf = new EventSpecificFeatures(4);

		// for testing, debugging and profiling purposes:
		esf.addKeyValuePair("event_reception_time", ""
				+ m_df.format(new Date()));
		esf.addKeyValuePair("x", "" + _node.x);
		esf.addKeyValuePair("y", "" + _node.y);

		addPartialEvent("FNode", EventSpecificBinaryDataIO
				.objectToByteArray(_node), time, time, null, esf);
	}
}
