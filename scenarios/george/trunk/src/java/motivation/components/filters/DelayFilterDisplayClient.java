package motivation.components.filters;

import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.HashMap;

import motivation.slice.Motive;
import motivation.util.WMMotiveView;
import si.unilj.fri.cogx.v11n.core.DisplayClient;
import cast.cdl.WorkingMemoryAddress;
import cast.core.Pair;

class DelayFilterDisplayClient extends DisplayClient {

	private static class PairComparator<A> implements Comparator<Pair<A, Long>> {

		@Override
		public int compare(Pair<A, Long> o1, Pair<A, Long> o2) {
			return (int) (o1.m_second - o2.m_second);
		}

	}

	private static final PairComparator<Motive> MOTIVE_PAIR_COMPARATOR = new PairComparator<Motive>();

	DelayFilterDisplayClient() {
	}

	
//private static String 
	
	public void updateDelayDisplay(long _currentSystemTime,
			HashMap<Class<? extends Motive>, Long> _activationTimes,
			HashMap<Class<? extends Motive>, Long> _postDelays,
			WMMotiveView _motives) {

		// delay table, sorted from bottom (longest delayed) to top (shortest
		// delay), undelayed or otherwise surfaced motives are not shown

		ArrayList<Pair<Motive, Long>> motiveDelays = new ArrayList<Pair<Motive, Long>>(
				_motives.size());

		for (WorkingMemoryAddress mtvAddr : _motives.keySet()) {
			Motive mtv = _motives.get(mtvAddr);

			Long delay = _activationTimes.get(mtv.getClass());
			if (delay != null) {
				motiveDelays.add(new Pair<Motive, Long>(mtv,
						(delay - _currentSystemTime)));
			}
		}

		Collections.sort(motiveDelays, MOTIVE_PAIR_COMPARATOR);

		// LinkedList<String> rows = new LinkedList<String>();
		String rows = "";

		for (Pair<Motive, Long> pair : motiveDelays) {
			rows += pair.m_first.getClass() + " " + pair.m_second + "<br/>";
		}
		setHtml("motive.filter", "002", rows);
	}
}