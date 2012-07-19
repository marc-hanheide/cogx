package motivation.components.filters;

import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.HashMap;
import java.util.LinkedList;
import java.util.Map;
import java.util.concurrent.ScheduledThreadPoolExecutor;
import java.util.concurrent.TimeUnit;

import org.apache.log4j.Logger;

import motivation.slice.AnalyzeProtoObjectMotive;
import motivation.slice.LearnObjectFeatureMotive;
import motivation.slice.LookAtViewConeMotive;
import motivation.slice.Motive;
import motivation.slice.RobotNonSituatedMotive;
import motivation.slice.TutorInitiativeMotive;
import motivation.util.WMMotiveView;
import si.unilj.fri.cogx.v11n.core.DisplayClient;
import cast.cdl.WorkingMemoryAddress;
import cast.core.CASTUtils;
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
				updateAll();
			} catch (Throwable t) {
				m_logger.warn("Nick, fix me", t);
			}
		}

	}

	private static final String ACTIVE_ID = "001";
	private static final String DRIVE_HIERARCHY_ID = "002";
	private static final String DELAYS_ID = "003";
	private static final String PASS_THROUGH_ID = "004";

	private static final String DISPLAY_ID = "motive.filter";

	private static final PairComparator<Motive> MOTIVE_PAIR_COMPARATOR = new PairComparator<Motive>();

	private static final String ROBOT_NON_SITUATED = "B6CEB6";
	private static final String EXPLORE = "936663";
	private static final String ROBOT_SITUATED = "C7B26F";
	private static final String TUTOR_DRIVEN = "6E8243";

	private static void cell(StringBuilder _sb, Object _content) {
		_sb.append("<td>");
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

	private static StringBuilder startFilter() {
		return new StringBuilder("<hr/><div>");
	}

	private static void startRow(StringBuilder _sb) {
		_sb.append("<tr>");
	}

	private static final HashMap<Class<? extends Motive>, String> m_colourMap = new HashMap<Class<? extends Motive>, String>();
	private static HashMap<Class<? extends Motive>, Integer> m_passThroughValues;

	private DriveHierarchy m_driveHierarchy;
	private long m_currentSystemTime;
	private HashMap<Class<? extends Motive>, Long> m_activationTimes;
	private HashMap<Class<? extends Motive>, Long> m_postDelays;
	private WMMotiveView m_motives;

	private static DelayFilterDisplayClient m_client;

	// MASSIVE HACK with static
	public static DelayFilterDisplayClient getClient() {
		if (m_client == null) {
			m_client = new DelayFilterDisplayClient();
		}
		return m_client;
	}

	private DelayFilterDisplayClient() {
		setupColours();
	}

	private static void endTable(StringBuilder _sb) {
		_sb.append("</table");
	}

	public void setDriveHierarchy(DriveHierarchy _driveHierarchy) {
		m_driveHierarchy = _driveHierarchy;
	}

	public void setPassThroughState(
			HashMap<Class<? extends Motive>, Integer> _values,
			WMMotiveView _motives) {

		// synchronized (m_motives) {
		m_passThroughValues = _values;
		m_motives = _motives;
		updateAll();
		// }
	}

	private HashMap<WorkingMemoryAddress, Motive> renderPassThroughState(
			HashMap<Class<? extends Motive>, Integer> _values,
			WMMotiveView _m_motives) {

		HashMap<WorkingMemoryAddress, Motive> passOn = new HashMap<WorkingMemoryAddress, Motive>();

		// go through all motives and display which are turned off here

		StringBuilder sb = startFilter();
		startTable(sb);

		LinkedList<Motive> stoppedMotives = new LinkedList<Motive>();

		for (Motive mtv : _m_motives.values()) {
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
				cell(sb, CASTUtils.toString(mtv.thisEntry));
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

	protected void setupColours() {
		m_colourMap.put(AnalyzeProtoObjectMotive.class, EXPLORE);
		m_colourMap.put(TutorInitiativeMotive.class, TUTOR_DRIVEN);
		m_colourMap.put(LearnObjectFeatureMotive.class, ROBOT_SITUATED);
		m_colourMap.put(LookAtViewConeMotive.class, ROBOT_SITUATED);
		m_colourMap.put(RobotNonSituatedMotive.class, ROBOT_NON_SITUATED);
	}

	private static void startRow(StringBuilder _sb,
			Class<? extends Motive> _mtvCls) {
		_sb.append("<tr");
		String hexCol = m_colourMap.get(_mtvCls);
		if (hexCol != null) {
			_sb.append(" bgcolor=\"#");
			_sb.append(hexCol);
			_sb.append("\"");
		}
		_sb.append(">");
	}

	private static void startTable(StringBuilder _sb) {
		_sb.append("<table border=\"1\">");
	}

	public void updateActiveGoals(Map<WorkingMemoryAddress, Motive> _activeGoals) {
		StringBuilder sb = startFilter();
		startTable(sb);
		if (!_activeGoals.isEmpty()) {
			for (Motive mtv : _activeGoals.values()) {
				Class<? extends Motive> mtvCls = mtv.getClass();
				startRow(sb, mtvCls);
				cell(sb, mtvCls.getSimpleName());
				cell(sb, CASTUtils.toString(mtv.thisEntry));
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

	public void updateActiveLevel(int _level) {
		if (_level == DriveHierarchy.UNKNOWN_CLASS_VALUE) {
			setHtml(DISPLAY_ID, DRIVE_HIERARCHY_ID, "Waiting for any input ");
		} else {
			setHtml(DISPLAY_ID, DRIVE_HIERARCHY_ID, "Active level is " + _level);
		}
	}

	private void updateAll() {
		synchronized (m_motives) {

			HashMap<WorkingMemoryAddress, Motive> passOn = null;

			if (m_passThroughValues != null) {
				passOn = renderPassThroughState(m_passThroughValues, m_motives);
			}

			if (m_activationTimes != null) {
				passOn = renderDelays(m_currentSystemTime, m_activationTimes,
						m_postDelays, passOn);
			}
		}
	}

	private HashMap<WorkingMemoryAddress, Motive> renderDelays(
			long _currentSystemTime,
			HashMap<Class<? extends Motive>, Long> _activationTimes,
			HashMap<Class<? extends Motive>, Long> _postDelays,
			HashMap<WorkingMemoryAddress, Motive> _motives) {
		HashMap<WorkingMemoryAddress, Motive> passOn = new HashMap<WorkingMemoryAddress, Motive>();
		synchronized (m_motives) {
			// delay table, sorted from bottom (longest delayed) to top
			// (shortest
			// delay), undelayed or otherwise surfaced motives are not shown

			ArrayList<Pair<Motive, Long>> motiveDelays = new ArrayList<Pair<Motive, Long>>(
					_motives.size());

			for (WorkingMemoryAddress mtvAddr : _motives.keySet()) {
				Motive mtv = _motives.get(mtvAddr);

				Long activationTime = _activationTimes.get(mtv.getClass());
				if (activationTime != null) {
					long delay = (activationTime - _currentSystemTime);
					if (delay > 0) {
						motiveDelays.add(new Pair<Motive, Long>(mtv,
								(activationTime - _currentSystemTime)));
					} else {
						passOn.put(mtvAddr, mtv);
					}
				}
			}

			Collections.sort(motiveDelays, MOTIVE_PAIR_COMPARATOR);

			StringBuilder sb = startFilter();
			startTable(sb);
			if (!motiveDelays.isEmpty()) {
				for (Pair<Motive, Long> pair : motiveDelays) {
					Motive mtv = pair.m_first;
					Class<? extends Motive> mtvCls = mtv.getClass();
					startRow(sb, mtvCls);
					cell(sb, mtvCls.getSimpleName());
					cell(sb, CASTUtils.toString(mtv.thisEntry));
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
		}
		return passOn;
	}

	public void updateDelays(long _currentSystemTime,
			HashMap<Class<? extends Motive>, Long> _activationTimes,
			HashMap<Class<? extends Motive>, Long> _postDelays,
			WMMotiveView _motives) {

		synchronized (m_motives) {
			m_currentSystemTime = _currentSystemTime;
			m_activationTimes = _activationTimes;
			m_postDelays = _postDelays;
			m_motives = _motives;
			updateAll();
		}

	}
}