/**
 * @author Marc Hanheide (marc@hanheide.de)
 */
package de.dfki.lt.tr.beliefs.data;

import cast.cdl.CASTTime;
import cast.core.CASTUtils;
import castutils.CASTTimeUtil;
import de.dfki.lt.tr.beliefs.data.genericproxies.GenericFrame;
import de.dfki.lt.tr.beliefs.slice.framing.AbstractFrame;
import de.dfki.lt.tr.beliefs.slice.framing.CASTTemporalInterval;
import de.dfki.lt.tr.beliefs.slice.framing.SpatioTemporalFrame;

/**
 * @author Marc Hanheide (marc@hanheide.de)
 * 
 */
public class CASTFrame extends GenericFrame<SpatioTemporalFrame> {

	public static final String LABEL4HERE = "here";

	public static CASTFrame create() {
		CASTTime now = CASTUtils.getTimeServer().getCASTTime();
		return create(now);
	}

	public static CASTFrame create(AbstractFrame abstractFrame) {
		return new CASTFrame(abstractFrame);
	}

	public static CASTFrame create(CASTTime startAndEnd) {
		return create(LABEL4HERE, startAndEnd, startAndEnd);
	}

	public static CASTFrame create(String place, CASTTime start, CASTTime end) {
		return new CASTFrame(new SpatioTemporalFrame(place,
				new CASTTemporalInterval(start, end), 1.0f));
	}

	protected CASTFrame(AbstractFrame content) {
		super(SpatioTemporalFrame.class, content);
		assert (content instanceof SpatioTemporalFrame);
		assert (((SpatioTemporalFrame) content).interval instanceof CASTTemporalInterval);
	}

	/**
	 * @return the time in ms since the beginning of this frame (compared to now)
	 */
	public long age() {
		CASTTemporalInterval ti = (CASTTemporalInterval) _content.interval;
		return CASTTimeUtil.diff(CASTUtils.getTimeServer()
				.getCASTTime(),ti.start);
	}

	/**
	 * @return the duration in ms of this frame
	 */
	public long duration() {
		CASTTemporalInterval ti = (CASTTemporalInterval) _content.interval;
		return CASTTimeUtil.diff(ti.end, ti.start);
	}

	public String getPlace() {
		return _content.place;
	}

	public CASTTime getStartTime() {
		return ((CASTTemporalInterval) _content.interval).start;
	}

	public CASTTime getEndTime() {
		return ((CASTTemporalInterval) _content.interval).end;
	}

	
	public void setPlace(String s) {
		_content.place = s;
	}

	public void setTime(CASTTime start, CASTTime end) {
		_content.interval = new CASTTemporalInterval(start, end);
	}

	public void setExistProb(double p) {
		_content.existProb=(float) p;
	}

	public double getExistProb() {
		return _content.existProb;
	}
	
}
