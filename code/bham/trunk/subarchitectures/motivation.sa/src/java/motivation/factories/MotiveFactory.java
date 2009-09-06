/**
 * 
 */
package motivation.factories;

import cast.cdl.CASTTime;
import cast.cdl.WorkingMemoryAddress;
import cast.core.CASTUtils;
import motivation.slice.Motive;
import motivation.slice.MotiveStatus;

/**
 * @author marc
 *
 */
public class MotiveFactory {
	public static Motive createMotive(WorkingMemoryAddress src) {
		CASTTime created = CASTUtils.getTimeServer().getCASTTime();
		return new Motive(created, created, src, MotiveStatus.UNSURFACED, "");
	}
}
