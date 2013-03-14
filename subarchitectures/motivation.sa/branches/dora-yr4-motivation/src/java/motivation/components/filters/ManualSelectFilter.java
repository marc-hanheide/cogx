package motivation.components.filters;

import motivation.slice.AnalyzeProtoObjectMotive;
import motivation.slice.CannedTextMotive;
import motivation.slice.CategorizeRoomMotive;
import motivation.slice.ExploreMotive;
import motivation.slice.HomingMotive;
import motivation.slice.MotivePriority;
import motivation.slice.PatrolMotive;
import motivation.slice.RobotInitiativeMotive;
import motivation.slice.RobotNonSituatedMotive;
import motivation.slice.TutorInitiativeMotive;

public class ManualSelectFilter extends AbstractManualSelectFilter {

	@Override
	protected void registerTypes() {
		addDefault("CogX default", ExploreMotive.class, MotivePriority.NORMAL);
		addDefault("CogX default", CategorizeRoomMotive.class,
				MotivePriority.NORMAL);
		addDefault("CogX default", HomingMotive.class, MotivePriority.UNSURFACE);
		addDefault("CogX default", PatrolMotive.class, MotivePriority.LOW);
		addDefault("CogX default", CannedTextMotive.class, MotivePriority.LOW);
		addDefault("CogX default", RobotInitiativeMotive.class,
				MotivePriority.NORMAL);
		addDefault("CogX default", TutorInitiativeMotive.class,
				MotivePriority.HIGH);
		addDefault("CogX default", RobotNonSituatedMotive.class,
				MotivePriority.LOW);
		addDefault("CogX default", AnalyzeProtoObjectMotive.class,
				MotivePriority.NORMAL);
		
		addDefault(STARTUP_PRIORITIES, TutorInitiativeMotive.class,
				MotivePriority.NORMAL);
		

	}

	public static void main(String[] args) {
		AbstractManualSelectFilter o = new ManualSelectFilter();
		o.start();
	}

}
