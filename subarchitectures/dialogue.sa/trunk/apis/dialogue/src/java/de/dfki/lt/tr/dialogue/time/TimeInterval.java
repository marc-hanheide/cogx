package de.dfki.lt.tr.dialogue.time;

import de.dfki.lt.tr.dialogue.util.ConvertibleToIce;
import de.dfki.lt.tr.dialogue.slice.time.Interval;

public class TimeInterval implements ConvertibleToIce<Interval> {

	private final Point begin;
	private final Point end;

	public TimeInterval(Point begin, Point end) {
		this.begin = begin;
		this.end = end;
	}

	public TimeInterval(Interval ival) {
		begin = new Point(ival.begin);
		end = new Point(ival.end);
	}

	public Interval toIce() {
		return new Interval(begin.toIce(), end.toIce());
	}

}
