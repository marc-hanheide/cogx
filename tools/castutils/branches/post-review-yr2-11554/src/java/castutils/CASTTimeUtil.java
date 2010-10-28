/**
 * 
 */
package castutils;

import cast.cdl.CASTTime;

/**
 * @author marc
 * 
 */
final public class CASTTimeUtil {
	public static long diff(CASTTime t1, CASTTime t2) {
		return (t1.s - t2.s) * 1000 + (t1.us - t2.us) / 1000;
	}

	public static String msString(CASTTime time) {
		return Long.toString(time.s*1000 + time.us/1000);
	}
}
