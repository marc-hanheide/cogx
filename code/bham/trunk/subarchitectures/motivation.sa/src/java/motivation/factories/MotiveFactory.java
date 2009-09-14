/**
 * 
 */
package motivation.factories;

import cast.cdl.CASTTime;
import cast.cdl.WorkingMemoryAddress;
import cast.core.CASTUtils;
import motivation.slice.ExploreMotive;
import motivation.slice.Motive;
import motivation.slice.MotiveStatus;
import motivation.slice.TestMotive;

/**
 * @author marc
 *
 */
public class MotiveFactory {
	public static Motive createMotive(WorkingMemoryAddress src) {
		CASTTime created = CASTUtils.getTimeServer().getCASTTime();
		return new Motive(created, created, src, null, MotiveStatus.UNSURFACED);
	}
	public static Motive createExploreMotive(WorkingMemoryAddress src) {
		CASTTime created = CASTUtils.getTimeServer().getCASTTime();
		return new ExploreMotive(created, created, src, null, MotiveStatus.UNSURFACED, -1);
	}
	public static Motive createTestMotive(WorkingMemoryAddress src) {
		CASTTime created = CASTUtils.getTimeServer().getCASTTime();
		return new TestMotive(created, created, src, null, MotiveStatus.UNSURFACED, "hurgs");
	}
}
