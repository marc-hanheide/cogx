/**
 * 
 */
package motivation.factories;

import motivation.slice.CategorizePlaceMotive;
import motivation.slice.CategorizeRoomMotive;
import motivation.slice.ExploreMotive;
import motivation.slice.GeneralGoalMotive;
import motivation.slice.HomingMotive;
import motivation.slice.Motive;
import motivation.slice.MotivePriority;
import motivation.slice.MotiveStatus;
import motivation.slice.PatrolMotive;
import motivation.slice.TestMotive;
import autogen.Planner.Goal;
import cast.cdl.CASTTime;
import cast.cdl.WorkingMemoryAddress;
import cast.core.CASTUtils;

/**
 * @author marc
 *
 */
public class MotiveFactory {
	public static Motive createMotive(WorkingMemoryAddress src) {
		CASTTime created = CASTUtils.getTimeServer().getCASTTime();
		return new Motive(created, created, src, null, null, MotiveStatus.UNSURFACED, 0, MotivePriority.UNSURFACE, 0, 0, 0, new Goal(-1, -1, "", false),0.0,10,60);
	}
	public static ExploreMotive createExploreMotive(WorkingMemoryAddress src) {
		CASTTime created = CASTUtils.getTimeServer().getCASTTime();
		return new ExploreMotive(created, created, src, null, null, MotiveStatus.UNSURFACED, 0, MotivePriority.UNSURFACE, 0, 0, 0,  new Goal(-1, -1, "", false),0.0,5,60,0);
	}
	public static CategorizePlaceMotive createCategorizePlaceMotive(WorkingMemoryAddress src) {
		CASTTime created = CASTUtils.getTimeServer().getCASTTime();
		return new CategorizePlaceMotive(created, created, src, null, null, MotiveStatus.UNSURFACED, 0, MotivePriority.UNSURFACE, 0, 0, 0, new Goal(-1, -1, "", false),0.0,5,180,0);
	}
	public static TestMotive createTestMotive(WorkingMemoryAddress src) {
		CASTTime created = CASTUtils.getTimeServer().getCASTTime();
		return new TestMotive(created, created, src, null, null, MotiveStatus.UNSURFACED, 0, MotivePriority.UNSURFACE, 0, 0, 0, new Goal(-1, -1, "", false),0.0,10,60,"hurgs");
	}
	public static HomingMotive createHomingMotive(WorkingMemoryAddress src) {
		CASTTime created = CASTUtils.getTimeServer().getCASTTime();
		return new HomingMotive(created, created, src, null, null, MotiveStatus.UNSURFACED, 0, MotivePriority.UNSURFACE, 0, 0, 0, new Goal(-1, -1, "", false),0.0,5,180,0);
	}
	public static CategorizeRoomMotive createCategorizeRoomMotive(WorkingMemoryAddress src) {
		CASTTime created = CASTUtils.getTimeServer().getCASTTime();
		return new CategorizeRoomMotive(created, created, src, null, null, MotiveStatus.UNSURFACED, 0, MotivePriority.UNSURFACE, 0, 0, 0, new Goal(-1, -1, "", false),0.0,5,180,-1);
	}
	public static PatrolMotive createPatrolMotive(WorkingMemoryAddress src) {
		CASTTime created = CASTUtils.getTimeServer().getCASTTime();
		return new PatrolMotive(created, created, src, null, null, MotiveStatus.UNSURFACED, 0, MotivePriority.UNSURFACE, 0, 0, 0,  new Goal(-1, -1, "", false),0.0,10,60,0, created);
	}
	public static GeneralGoalMotive createGeneralGoalMotive(WorkingMemoryAddress src) {
		CASTTime created = CASTUtils.getTimeServer().getCASTTime();
		return new GeneralGoalMotive(created, created, src, null, null, MotiveStatus.UNSURFACED, 0, MotivePriority.UNSURFACE, 0, 0, 0,  new Goal(-1, -1, "", false),0.0,10,Integer.MAX_VALUE);
	}
}
