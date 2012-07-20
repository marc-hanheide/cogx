package motivation.components.filters;

import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.HashMap;
import java.util.Iterator;
import java.util.LinkedList;
import java.util.Map;
import java.util.concurrent.ScheduledThreadPoolExecutor;
import java.util.concurrent.TimeUnit;

import motivation.slice.AnalyzeProtoObjectMotive;
import motivation.slice.ComplexActionCommandMotive;
import motivation.slice.LearnObjectFeatureMotive;
import motivation.slice.LookAtViewConeMotive;
import motivation.slice.Motive;
import motivation.slice.MotiveStatus;
import motivation.slice.RobotNonSituatedMotive;
import motivation.slice.TutorInitiativeLearningMotive;
import motivation.slice.TutorInitiativeMotive;
import motivation.slice.TutorInitiativeQuestionMotive;
import motivation.util.WMMotiveView;

import org.apache.log4j.Logger;

import si.unilj.fri.cogx.v11n.core.DisplayClient;
import cast.cdl.WorkingMemoryAddress;
import cast.core.CASTComponent;
import cast.core.Pair;

public class DelayFilterDisplayClient extends DisplayClient {

	private static Logger m_logger = Logger
			.getLogger(DelayFilterDisplayClient.class);

	private static final ScheduledThreadPoolExecutor m_executor = new ScheduledThreadPoolExecutor(
			1);

	private static class PairComparator<A> implements Comparator<Pair<A, Long>> {

		@Override
		public int compare(Pair<A, Long> o1, Pair<A, Long> o2) {
			return (int) (o1.m_second - o2.m_second);
		}

	}

	private class RenderThread implements Runnable {

		@Override
		public void run() {
			try {
				renderAll();
			} catch (Throwable t) {
				m_logger.warn("Nick, fix me", t);
			}
		}
	}

	private static final String KEY_ID = "007";
	private static final String ACTIVE_ID = "002";
	private static final String DRIVE_HIERARCHY_ID = "003";
	private static final String DELAYS_ID = "004";
	private static final String PASS_THROUGH_ID = "005";
	private static final String COMPLETED_ID = "006";

	private static final String DISPLAY_ID = "motive.filter";

	private static final PairComparator<Motive> MOTIVE_PAIR_COMPARATOR = new PairComparator<Motive>();

	private static final String ROBOT_NON_SITUATED = "B6CEB6";
	// private static final String EXPLORE = "936663";

	private static final String ROBOT_SITUATED = "C7B26F";
	private static final String EXPLORE = ROBOT_SITUATED;
	private static final String TUTOR_DRIVEN = "6E8243";

	private static void cell(StringBuilder _sb, Object _content) {
		_sb.append("<td>");
		_sb.append(_content.toString());
		_sb.append("</td>");
	}

	private static void cell(StringBuilder _sb, Object _content, String _hex) {
		_sb.append("<td bgcolor=\"#");
		_sb.append(_hex);
		_sb.append("\">");
		_sb.append(_content.toString());
		_sb.append("</td>");
	}

	private static void cell(StringBuilder _sb, Object _content, int _colspan) {
		_sb.append("<td colspan=\"");
		_sb.append(_colspan);
		_sb.append("\">");
		_sb.append(_content.toString());
		_sb.append("</td>");
	}

	private static void endFilter(StringBuilder _sb) {
		_sb.append("</div>");
	}

	private static void endRow(StringBuilder _sb) {
		_sb.append("</tr>");
	}

	// private static String

	private static StringBuilder startFilter(String _title) {
		StringBuilder sb = new StringBuilder("<div><h3>");
		sb.append(_title);
		sb.append("</h3>");
		return sb;
	}

	private static void startRow(StringBuilder _sb) {
		_sb.append("<tr>");
	}

	private static final HashMap<Class<? extends Motive>, String> m_colourMap = new HashMap<Class<? extends Motive>, String>();

	private static final String DISPLAY_CSS = "tr {font-size: 14px}\n";

	private static HashMap<Class<? extends Motive>, Integer> m_passThroughValues;

	private DriveHierarchy m_driveHierarchy;
	private long m_currentSystemTime;
	private HashMap<WorkingMemoryAddress, Long> m_activationTimes;
	private WMMotiveView m_motives = null;

	private Map<WorkingMemoryAddress, Motive> m_activeGoals;

	private int m_activeLevel = DriveHierarchy.UNKNOWN_CLASS_VALUE;

	private boolean m_connected = false;

	private static DelayFilterDisplayClient m_client;

	// MASSIVE HACK with static
	public static DelayFilterDisplayClient getClient(Map<String, String> _config) {
		if (m_client == null) {
			m_client = new DelayFilterDisplayClient();
			m_client.configureDisplayClient(_config);
		}
		return m_client;
	}

	private DelayFilterDisplayClient() {
		setupColours();
		m_executor.scheduleAtFixedRate(new RenderThread(), 10000, 1500,
				TimeUnit.MILLISECONDS);
	}

	private static void endTable(StringBuilder _sb) {
		_sb.append("</table>");
	}

	public synchronized void setDriveHierarchy(DriveHierarchy _driveHierarchy) {
		m_driveHierarchy = _driveHierarchy;
	}

	public synchronized void setPassThroughState(
			HashMap<Class<? extends Motive>, Integer> _values,
			WMMotiveView _motives) {

		m_passThroughValues = _values;
		m_motives = _motives;
	}

	private synchronized HashMap<WorkingMemoryAddress, Motive> renderPassThroughState(
			HashMap<Class<? extends Motive>, Integer> _values,
			HashMap<WorkingMemoryAddress, Motive> _motives) {

		HashMap<WorkingMemoryAddress, Motive> passOn = new HashMap<WorkingMemoryAddress, Motive>();

		// go through all motives and display which are turned off here

		StringBuilder sb = startFilter("Filter Override");
		startTable(sb);

		LinkedList<Motive> stoppedMotives = new LinkedList<Motive>();

		for (Motive mtv : _motives.values()) {
			Integer value = _values.get(mtv.getClass());
			// if this is managed by the slide and it's turned off
			if (value != null && value == 0) {
				stoppedMotives.add(mtv);
			} else {
				passOn.put(mtv.thisEntry, mtv);
			}
		}

		if (!stoppedMotives.isEmpty()) {
			for (Motive mtv : stoppedMotives) {
				Class<? extends Motive> mtvCls = mtv.getClass();
				startRow(sb, mtvCls);
				cell(sb, mtvCls.getSimpleName());
				// cell(sb, CASTUtils.toString(mtv.thisEntry));
				cell(sb, mtv.goal.goalString);
				endRow(sb);
			}
		} else {
			startRow(sb);
			cell(sb, "No manually stopped goals");
			endRow(sb);
		}
		endTable(sb);
		endFilter(sb);
		setHtml(DISPLAY_ID, PASS_THROUGH_ID, sb.toString());
		return passOn;
	}

	protected synchronized void setupColours() {
		m_colourMap.put(AnalyzeProtoObjectMotive.class, EXPLORE);
		m_colourMap.put(TutorInitiativeMotive.class, TUTOR_DRIVEN);
		m_colourMap.put(ComplexActionCommandMotive.class, TUTOR_DRIVEN);
		m_colourMap.put(TutorInitiativeLearningMotive.class, TUTOR_DRIVEN);
		m_colourMap.put(TutorInitiativeQuestionMotive.class, TUTOR_DRIVEN);
		m_colourMap.put(LearnObjectFeatureMotive.class, ROBOT_SITUATED);
		m_colourMap.put(LookAtViewConeMotive.class, ROBOT_SITUATED);
		m_colourMap.put(RobotNonSituatedMotive.class, ROBOT_NON_SITUATED);
	}

	private static void startRow(StringBuilder _sb,
			Class<? extends Motive> _mtvCls) {
		String hexCol = m_colourMap.get(_mtvCls);
		if (hexCol != null) {
			startRow(_sb, hexCol);
		} else {
			startRow(_sb);
		}
	}

	private static void startRow(StringBuilder _sb, String _hex) {
		_sb.append("<tr");
		_sb.append(" bgcolor=\"#");
		_sb.append(_hex);
		_sb.append("\"");
		_sb.append(">");
	}

	private static void startTable(StringBuilder _sb) {
		_sb.append("<table frame=\"border\" border=\"1\" rules=\"all\">");
	}

	private synchronized void renderActiveGoals(
			Map<WorkingMemoryAddress, Motive> _activeGoals) {
		StringBuilder sb = startFilter("Active Goals");
		startTable(sb);
		if (!_activeGoals.isEmpty()) {
			for (Motive mtv : _activeGoals.values()) {
				Class<? extends Motive> mtvCls = mtv.getClass();
				startRow(sb, mtvCls);
				cell(sb, mtvCls.getSimpleName());
				// cell(sb, CASTUtils.toString(mtv.thisEntry));
				cell(sb, mtv.goal.goalString);
				endRow(sb);
			}
		} else {
			startRow(sb);
			cell(sb, "No active goals");
			endRow(sb);
		}
		endTable(sb);
		endFilter(sb);
		setHtml(DISPLAY_ID, ACTIVE_ID, sb.toString());
	}

	public synchronized void updateActiveGoals(
			Map<WorkingMemoryAddress, Motive> _activeGoals) {
		m_activeGoals = _activeGoals;
	}

	public synchronized void updateActiveLevel(int _level) {
		m_activeLevel = _level;
	}

	private synchronized void renderDriveHierarchy(
			HashMap<WorkingMemoryAddress, Motive> _motives) {

		StringBuilder sb = startFilter("Drive Hierarchy");

		// if (m_activeLevel == DriveHierarchy.UNKNOWN_CLASS_VALUE) {
		// sb.append("Waiting for any input ");
		// } else {
		// sb.append("Active level is " + m_activeLevel);
		// }

		int driveLevels = m_driveHierarchy.size();

		boolean drawnThreshold = false;

		startTable(sb);

		for (int i = 0; i < driveLevels; i++) {
			startRow(sb);
			cell(sb, "Drive Level: " + i, 2);
			endRow(sb);

			Iterator<WorkingMemoryAddress> iterator = _motives.keySet()
					.iterator();
			while (iterator.hasNext()) {
				WorkingMemoryAddress mtvAddr = iterator.next();
				Motive mtv = _motives.get(mtvAddr);
				Class<? extends Motive> mtvCls = mtv.getClass();
				// if this motive belongs at this level
				if (m_driveHierarchy.getPriority(mtv.getClass()) == i) {
					startRow(sb, mtvCls);
					cell(sb, mtvCls.getSimpleName());
					// cell(sb, CASTUtils.toString(mtv.thisEntry));
					cell(sb, mtv.goal.goalString);
					endRow(sb);
					m_logger.info("motives size before: " + _motives.size());
					iterator.remove();
					m_logger.info("motives size after: " + _motives.size());
				}

			}

			if (i == m_activeLevel) {
				startRow(sb, "000000");
				cell(sb, "Threshold", 2);
				endRow(sb);
				drawnThreshold = true;
			}

		}

		if (!drawnThreshold) {
			startRow(sb, "000000");
			cell(sb, "Threshold", 2);
			endRow(sb);
			drawnThreshold = true;
		}
		endTable(sb);

		endFilter(sb);
		setHtml(DISPLAY_ID, DRIVE_HIERARCHY_ID, sb.toString());

	}

	private synchronized void renderAll() {

		

		if (m_motives != null) {

			HashMap<WorkingMemoryAddress, Motive> passOn = renderCompleted(m_motives);

			if (m_passThroughValues != null) {
				// m_logger.info("pass through: " + passOn.size());
				passOn = renderPassThroughState(m_passThroughValues, passOn);
			}

			if (m_activationTimes != null) {
				// m_logger.info("activations: " + passOn.size());
				passOn = renderDelays(m_currentSystemTime, m_activationTimes,
						passOn);
			}

			if (m_driveHierarchy != null) {
				// m_logger.info("drive hierarchy: " + passOn.size());
				renderDriveHierarchy(passOn);
			}

			// if (m_activeGoals != null) {
			// // TODO use passOn once it makes sense
			// m_logger.info("drive hierarchy: " + m_activeLevel);
			// renderActiveGoals(m_activeGoals);
			// }
		}
		
		renderKey();
	}

	// private static final String ROBOT_NON_SITUATED = "B6CEB6";
	// private static final String EXPLORE = "936663";
	// private static final String ROBOT_SITUATED = "C7B26F";
	// private static final String TUTOR_DRIVEN = "6E8243";
	//

	private synchronized void renderKey() {
		StringBuilder sb = startFilter("Key");
		startTable(sb);

		startRow(sb);
		cell(sb, "&nbsp &nbsp &nbsp &nbsp", TUTOR_DRIVEN);
		cell(sb, "Interactive");
		endRow(sb);

		startRow(sb);
		cell(sb, "&nbsp &nbsp &nbsp &nbsp", ROBOT_SITUATED);
		cell(sb, "Extrospective");
		endRow(sb);

		// startRow(sb);
		// cell(sb, "&nbsp &nbsp &nbsp &nbsp", EXPLORE);
		// cell(sb, "Extrospective (New Objects)");
		// endRow(sb);

		startRow(sb);
		cell(sb, "&nbsp &nbsp &nbsp &nbsp", ROBOT_NON_SITUATED);
		cell(sb, "Introspective");
		endRow(sb);

		endTable(sb);
		endFilter(sb);
		setHtml(DISPLAY_ID, KEY_ID, sb.toString());

	}

	private synchronized HashMap<WorkingMemoryAddress, Motive> renderCompleted(
			WMMotiveView _motives) {

		HashMap<WorkingMemoryAddress, Motive> passOn = new HashMap<WorkingMemoryAddress, Motive>();

		for (Motive mtv : _motives.values()) {
			if (mtv.status != MotiveStatus.COMPLETED && mtv.tries < 4) {
				passOn.put(mtv.thisEntry, mtv);
			}
		}

		return passOn;
	}

	private synchronized HashMap<WorkingMemoryAddress, Motive> renderDelays(
			long _currentSystemTime,
			HashMap<WorkingMemoryAddress, Long> _activationTimes,
			HashMap<WorkingMemoryAddress, Motive> _motives) {

		HashMap<WorkingMemoryAddress, Motive> passOn = new HashMap<WorkingMemoryAddress, Motive>();
		// delay table, sorted from bottom (longest delayed) to top
		// (shortest
		// delay), undelayed or otherwise surfaced motives are not shown

		ArrayList<Pair<Motive, Long>> motiveDelays = new ArrayList<Pair<Motive, Long>>(
				_motives.size());

		for (WorkingMemoryAddress mtvAddr : _motives.keySet()) {
			Motive mtv = _motives.get(mtvAddr);

			Long activationTime = _activationTimes.get(mtv.thisEntry);
			if (activationTime != null) {
				long delay = (activationTime - _currentSystemTime);
				if (delay > 0) {
					motiveDelays.add(new Pair<Motive, Long>(mtv,
							(activationTime - _currentSystemTime)));
				} else {
					passOn.put(mtvAddr, mtv);
				}
			} else {
				passOn.put(mtvAddr, mtv);
			}
		}

		Collections.sort(motiveDelays, MOTIVE_PAIR_COMPARATOR);

		StringBuilder sb = startFilter("Delay Filter");
		startTable(sb);
		if (!motiveDelays.isEmpty()) {
			for (Pair<Motive, Long> pair : motiveDelays) {
				Motive mtv = pair.m_first;
				Class<? extends Motive> mtvCls = mtv.getClass();
				startRow(sb, mtvCls);
				cell(sb, mtvCls.getSimpleName());
				// cell(sb, CASTUtils.toString(mtv.thisEntry));
				cell(sb, pair.m_second);
				cell(sb, mtv.goal.goalString);
				endRow(sb);
			}
		} else {
			startRow(sb);
			cell(sb, "No delayed goals");
			endRow(sb);
		}
		endTable(sb);
		endFilter(sb);
		setHtml(DISPLAY_ID, DELAYS_ID, sb.toString());
		return passOn;
	}

	public synchronized void updateDelays(long _currentSystemTime,
			HashMap<WorkingMemoryAddress, Long> _activeDelays) {

		m_currentSystemTime = _currentSystemTime;
		m_activationTimes = _activeDelays;

	}

	public void connect(CASTComponent _component) {
		if (!m_connected) {
			connectIceClient(_component);
			m_connected = true;
			setHtmlHead(DISPLAY_ID, "head", "<style type=\"text/css\">"
					+ DISPLAY_CSS + "</style>");
		}

	}
}