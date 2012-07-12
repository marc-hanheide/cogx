package motivation.components.filters;

import motivation.slice.AnalyzeProtoObjectMotive;
import motivation.slice.LearnObjectFeatureMotive;
import motivation.slice.LookAtViewConeMotive;
import motivation.slice.RobotNonSituatedMotive;
import motivation.slice.TutorInitiativeLearningMotive;
import motivation.slice.TutorInitiativeQuestionMotive;

public abstract class GeorgeDriveConfig {

	private static DriveHierarchy m_driveHierarchy;

	@SuppressWarnings("unchecked")
	public static DriveHierarchy getGeorgeDriveHierarchy() {
		if (m_driveHierarchy == null) {

			m_driveHierarchy = new DriveHierarchy();

			m_driveHierarchy.addPrioritySet(
					TutorInitiativeLearningMotive.class,
					TutorInitiativeQuestionMotive.class);

			m_driveHierarchy.addPrioritySet(AnalyzeProtoObjectMotive.class,
					LearnObjectFeatureMotive.class);

			m_driveHierarchy.addPrioritySet(LookAtViewConeMotive.class);

			m_driveHierarchy.addPrioritySet(RobotNonSituatedMotive.class);
		}

		return m_driveHierarchy;
	}

}
