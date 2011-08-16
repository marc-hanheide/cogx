package autoCostEst;

import java.io.Serializable;
import java.util.Calendar;
import java.util.Date;

/**
 * represents a day's worth of timings over a particular path (and previous
 * path)
 * 
 * @author ken
 * 
 */
public class Timings implements Serializable {

	/**
	 * 
	 */
	private static final long serialVersionUID = 1L;
	private int previous;// the node that the robot came from
	private int startPoint, endPoint;// the start and end points of the path
	// being measured
	private int[][][] times;

	public Timings(int previous, int startPoint, int endPoint) {
		this.previous = previous;
		this.startPoint = startPoint;
		this.endPoint = endPoint;
		times = new int[7][24][6];

	}

	public Timings(long previous, long startPoint, long endPoint) {
		this((int) previous, (int) startPoint, (int) endPoint);

	}

	public int getPrev() {
		return previous;
	}

	public int getStartPoint() {
		return startPoint;
	}

	public int getEndPoint() {
		return endPoint;
	}

	/**
	 * returns the timing associated with the current time;
	 * 
	 * @return
	 */
	public int getCurrentTiming() {

		Calendar cal = Calendar.getInstance();
		cal.setTime(new Date(System.currentTimeMillis()));

		return getTiming(cal.get(Calendar.DAY_OF_WEEK), cal
				.get(Calendar.HOUR_OF_DAY), cal.get(Calendar.MINUTE));
	}

	public void setTiming(int day, int hour, int mins, int time){
		times[day][hour][mins]=time;
	}
	public int getTimingFromArray(int day, int hour, int mins){
		return times[day][hour][mins];
	}
	
	public void setCurrentTiming(int newTime) {
		Calendar cal = Calendar.getInstance();
		cal.setTime(new Date(System.currentTimeMillis()));
		int day = cal.get(Calendar.DAY_OF_WEEK);
		int hour = cal.get(Calendar.HOUR_OF_DAY);
				int mins = cal.get(Calendar.MINUTE);
		hour -= 1;// (we start at 0 because we're using an array)
		if (!(mins > 54)) {
			mins = mins / 5;// break into 5 minute increments
			mins = (mins + 1) / 2; // where the increment sits in the array
		} else {
			mins = 0;
			hour = hour + 1; // keeps the 55-60 range with the 0-5 range of the
			// next hour
		}
		times[day][hour][mins] = newTime;
	}

	/**
	 * return the timing from a specified number of seconds in the future
	 * 
	 * @param mins
	 * @return
	 */
	public int getTimingIn(int seconds) {
		Calendar cal = Calendar.getInstance();
		cal.setTime(new Date(System.currentTimeMillis() + (seconds * 1000)));
		return getTiming(cal.get(Calendar.DAY_OF_WEEK), cal
				.get(Calendar.HOUR_OF_DAY), cal.get(Calendar.MINUTE));
	}

	/**
	 * given a time and hour, will return the timing for that
	 * 
	 * @param hour
	 * @param mins
	 * @return
	 */
	private int getTiming(int day, int hour, int mins) {
		assert ((day >= 0) && day < 7);
		assert ((hour >= 0) && hour < 24);
		assert ((mins >= 0) && (mins < 60));
		hour -= 1;// (we start at 0 because we're using an array)
		if (!(mins > 54)) {
			mins = mins / 5;// break into 5 minute increments
			mins = (mins + 1) / 2; // where the increment sits in the array
		} else {
			mins = 0;
			hour = hour + 1; // keeps the 55-60 range with the 0-5 range of the
			// next hour
			if(hour==24){
				hour=0;
				day+=1;
				if(day ==7){
					day=0;
				}
			}
		}

		return times[day][hour][mins];
	}

	/**
	 * returns how long until the robot is in the next time period in minutes
	 * 
	 * @return how long until the next time period
	 */
	public static int minutesTillNextPeriod() {

		Calendar cal = Calendar.getInstance();
		cal.setTime(new Date(System.currentTimeMillis()));
		int current = cal.get(Calendar.MINUTE);
		int next = current;
		if (current > 5) {
			if (current > 15) {
				if (current > 25) {
					if (current > 35) {
						if (current > 45) {
							if (current > 55) {
								next = 64;
							} else {
								next = 54;
							}
						} else {
							next = 44;
						}
					} else {
						next = 34;
					}
				} else {
					next = 24;
				}
			} else {
				next = 14;
			}
		} else {
			next = 4;
		}
		return next - current;
	}

	public static int timeTillNextPeriod() {
		int mins = minutesTillNextPeriod();
		Calendar cal = Calendar.getInstance();
		cal.setTime(new Date(System.currentTimeMillis()));
		int current = cal.get(Calendar.SECOND);
		current = 60 - current;
		int returnV = (mins*60)+current+1;
		if(returnV ==0){
			return 600;
		}
		return returnV;
	}

	@Override
	public String toString() {
		String ret = "";
		ret+=("timings for ");
		ret+=("prev: " + previous + " startPoint " + startPoint
				+ " endPoint " + endPoint);

		for (int day = 0; day < 7; day++) {
			ret+=(" \n day: " + day);
			for (int hour = 0; hour < 24; hour++) {
				ret+=(" \n hour " + hour);
				for (int minute = 0; minute < 6; minute++) {
					ret+=(" minute: " + minute + " is "
							+ times[day][hour][minute]);
				}
			}

		}
		ret+=("\n");
		return ret;

	}

}
