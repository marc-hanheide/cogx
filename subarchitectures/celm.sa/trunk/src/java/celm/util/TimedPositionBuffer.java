/** _
 *  part of the CELM system.
 *  @author Dennis Stachowicz
 */

package celm.util;

import elm.util.CircularBuffer;

/**
 * This is the main data structure in which LocationMonitor holds timestamped
 * positional information for its information fusion task. Insertion works in
 * O(1), retrieval of a position matching a given time in O(log buffer.size()).
 */
public class TimedPositionBuffer {

	private class TimedPositionBufferImpl extends CircularBuffer<TimedPosition> {

		public TimedPositionBufferImpl(int capacity)
				throws ArrayIndexOutOfBoundsException {

			super(capacity);
		}

	};

	private TimedPositionBufferImpl buffer;
	private long recent = -1;

	public TimedPositionBuffer(int capacity)
			throws ArrayIndexOutOfBoundsException {

		buffer = new TimedPositionBufferImpl(capacity);
	}

	public long getEarliestTimeValue() {
		if (buffer.size() > 0)
			return buffer.get(0).getTime();
		else
			return -1;
	}

	public long getLatestTimeValue() {
		return recent;
	}

	public int size() {
		return buffer.size();
	}

	public int capacity() {
		return buffer.capacity();
	}

	public boolean full() {
		return buffer.full();
	}

	public boolean empty() {
		return buffer.empty();
	}

	synchronized public void addPosition(TimedPosition tp) {

		long t = tp.getTime();
		if (t >= recent) {
			buffer.addOrOverwrite(tp);
			recent = t;
		}
		// else
		// throw an exception or just silently ignore?
	}

	synchronized private int getIndex(long t) throws PositionNotFoundException {

		if (empty())
			throw new PositionNotFoundException("buffer is empty!");
		if (t < buffer.get(0).getTime())
			throw new PositionNotFoundException(
					PositionNotFoundException.TOO_LATE);
		if (t > recent)
			throw new PositionNotFoundException(
					PositionNotFoundException.TOO_EARLY);

		int low = 0;
		int high = buffer.size() - 1;
		int mid = (high + low) / 2;

		while (high - low > 1) {

			if (t == buffer.get(mid).getTime())
				return mid;
			else if (t < buffer.get(mid).getTime()) {
				high = mid;
				mid = (high + low) / 2;
			} else {
				low = mid;
				mid = (high + low) / 2;
			}
		}

		if (Math.abs(t - buffer.get(low).getTime()) < Math.abs(t
				- buffer.get(high).getTime()))
			return low;
		else
			return high;
	}

	// ----- DEBUG ----
	/*
	 * synchronized private String bufferElementsToString() { StringBuffer sbuf
	 * = new StringBuffer(); for (int i = 0; i < buffer.size(); i++)
	 * sbuf.append(i + ": " + buffer.get(i) + "\n"); return sbuf.toString(); }
	 */

	// do not call with end > getLatestTimeValue()!
	synchronized public double[][] getPositionArray(long begin, long end)
			throws PositionNotFoundException {

		int beginIndex = getIndex(begin);
		int endIndex = getIndex(end);
		int size = endIndex - beginIndex + 1;

		// ----- DEBUG ----
		/*
		 * System.err.println("getPositionArray:" + "\nearliest: " +
		 * getEarliestTimeValue() + "\nlatest:   " + getLatestTimeValue() +
		 * "\nbegin:    " + begin + "\nend:      " + end + "\n\nbuffer:\n" +
		 * bufferElementsToString() + "\n\nfound:\n" + "\nbegin:    " +
		 * beginIndex + " - " + buffer.get(beginIndex).getTime() +
		 * "\nend:      " + endIndex + " - " + buffer.get(endIndex).getTime());
		 */
		// ----- DEBUG ----
		double[][] coordinatesArray = new double[size][2];

		// for very high numbers of available positions some reduction
		// or filtering might be needed...
		for (int i = 0; i < size; i++) {
			// coordinatesArray[i] = new double[2];
			
//			assert false : "need to put back in lines below";
			//coordinatesArray[i][0] = buffer.get(beginIndex + i).getRobotPose().x;
			//coordinatesArray[i][1] = buffer.get(beginIndex + i).getRobotPose().y;
		}

		return coordinatesArray;
	}

}
