/**
 * 
 */
package motivation.factories;

import cast.cdl.CASTTime;
import cast.cdl.WorkingMemoryAddress;
import cast.core.CASTUtils;
import motivation.slice.ExploreMotive;
import motivation.slice.HomingMotive;
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
		return new Motive(created, created, src, null, MotiveStatus.UNSURFACED, 0, 0, 0, 0, 0);
	}
	public static ExploreMotive createExploreMotive(WorkingMemoryAddress src) {
		CASTTime created = CASTUtils.getTimeServer().getCASTTime();
		return new ExploreMotive(created, created, src, null, MotiveStatus.UNSURFACED, 0, 0, 0, 0, 0, 0);
	}
	public static TestMotive createTestMotive(WorkingMemoryAddress src) {
		CASTTime created = CASTUtils.getTimeServer().getCASTTime();
		return new TestMotive(created, created, src, null, MotiveStatus.UNSURFACED, 0, 0, 0, 0, 0, "hurgs");
	}
	public static HomingMotive createHomingMotive(WorkingMemoryAddress src) {
		CASTTime created = CASTUtils.getTimeServer().getCASTTime();
		return new HomingMotive(created, created, src, null, MotiveStatus.UNSURFACED, 0, 0, 0, 0, 0, 0);
	}
}
