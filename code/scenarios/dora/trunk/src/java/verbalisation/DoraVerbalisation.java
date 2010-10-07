package verbalisation;

import java.util.Map;

import motivation.slice.CategorizeRoomMotive;
import motivation.slice.ExploreMotive;
import motivation.slice.Motive;
import motivation.slice.MotiveStatus;
import motivation.slice.PatrolMotive;
import SpatialData.NavCommand;
import SpatialData.Place;
import SpatialData.PlaceStatus;
import SpatialProperties.GatewayPlaceProperty;
import SpatialProperties.ObjectPlaceProperty;
import VisionData.DetectionCommand;
import VisionData.VisualObject;
import cast.architecture.ManagedComponent;
import castutils.castextensions.Accessor;



/**
 * - found new place N - found new placeholder N - found new room N - seen
 * object O - room N is categorised as a C - found a doorway - going to place N
 * - going room categorise room b - activated goals G - planning for active
 * goals - planning succeeded/failed - executing plan - executing next action -
 * action succeeded/failed - plan execution succeeded/failed
 * 
 * 
 * subarchitectures/comsys/grammars/contentPlanning/doc/cogx-testbed.xml
 * 
 * @author nah
 * 
 */
public class DoraVerbalisation extends ManagedComponent {

	private static final TextGenerator<NavCommand> NAV_CMD_GENERATOR = new TextGenerator<NavCommand>() {
		@Override
		public String toText(NavCommand _i) {
			switch (_i.cmd) {
			case GOTOPLACE:
				return "Going to Place " + _i.destId[0];
			case TURNTO:
				return "Turning";

			default:
				// return "I'm doing something, but I don't want talk about it";
				return "";
			}
		}
	};

	private static final TextGenerator<Motive> MOTIVE_ACTIVATED = new TextGenerator<Motive>() {
		@Override
		public String toText(Motive _i) {

			if (_i instanceof ExploreMotive) {
				return "Activated motive to explore hypothesis for Place "
						+ ((ExploreMotive) _i).placeID;
			} else if (_i instanceof CategorizeRoomMotive) {
				return "Activated motive to determine category of room "
						+ ((CategorizeRoomMotive) _i).roomId;
			} else if (_i instanceof PatrolMotive) {
				return "Activated motive to patrol place "
						+ ((PatrolMotive) _i).placeID;
			}
			return "";
		}
	};

	private static final String[] DETECTION_COMMAND_SAYINGS = {
			"Looking here.", "Having a look-see", "What have we here" };

	private static final TextGenerator<DetectionCommand> DETECTION_COMMAND_GENERATOR = new TextGenerator<DetectionCommand>() {
		private int m_sayingIndex = 0;

		@Override
		public String toText(DetectionCommand _i) {
			if (m_sayingIndex == DETECTION_COMMAND_SAYINGS.length) {
				m_sayingIndex = 0;
			}
			return DETECTION_COMMAND_SAYINGS[m_sayingIndex++];
		}
	};

	private static final TextGenerator<ObjectPlaceProperty> OBJECT_PROPERTY_GENERATOR = new TextGenerator<ObjectPlaceProperty>() {
		@Override
		public String toText(ObjectPlaceProperty _i) {
			return "I see something that appears to be "
					+ ((SpatialProperties.StringValue) _i.mapValue).value;
		}
	};

	private static final TextGenerator<VisualObject> VISUAL_OBJECT_GENERATOR = new TextGenerator<VisualObject>() {
		@Override
		public String toText(VisualObject _i) {
			if (_i.detectionConfidence > 0.5) {
				return "That looks like " + _i.identLabels[0];
			} else {
				return "";
			}
		}
	};

	private static Accessor<Motive, MotiveStatus> MOTIVE_STATUS_ACCESSOR = new Accessor<Motive, MotiveStatus>() {
		@Override
		public MotiveStatus access(Motive _entry) {
			return _entry.status;
		}
	};

	private static Accessor<Place, PlaceStatus> PLACE_STATUS_ACCESSOR = new Accessor<Place, PlaceStatus>() {
		@Override
		public PlaceStatus access(Place _entry) {
			return _entry.status;
		}
	};

	private static TextGenerator<Place> NEW_PLACE_GENERATOR = new TextGenerator<Place>() {

		@Override
		public String toText(Place _i) {

			StringBuilder sb = new StringBuilder();
			if (_i.status == PlaceStatus.PLACEHOLDER) {
				sb.append("I have a hypothesis that Place ");
				sb.append(_i.id);
				sb.append("exists");
			} else {
				sb.append("Created Place ");
				sb.append(_i.id);
			}
			return sb.toString();
		}
	};

	private static TextGenerator<Place> PLACE_EXPLORED_GENERATOR = new TextGenerator<Place>() {
		@Override
		public String toText(Place _i) {
			StringBuilder sb = new StringBuilder();
			sb.append("Created Place ");
			sb.append(_i.id);
			sb.append(" from an existing hypothesis");
			return sb.toString();
		}
	};

	private static TextGenerator<Place> PLACE_EXPLORATION_FAILED_GENERATOR = new TextGenerator<Place>() {
		@Override
		public String toText(Place _i) {
			StringBuilder sb = new StringBuilder();
			sb.append("Hypothesis for Place ");
			sb.append(_i.id);
			sb.append(" was not valid");
			return sb.toString();
		}
	};

	private final VerbalisationFacade m_verbals;

	public DoraVerbalisation() {
		m_verbals = new VerbalisationFacade(this);
	}

	@Override
	public void configure(Map<String, String> _config) {
		m_verbals.configure(_config);
	}

	public void start() {

		// say stuff...

		// when all motives are activated
		m_verbals.verbaliseOnStateTransition(Motive.class,
				MOTIVE_STATUS_ACCESSOR, MotiveStatus.SURFACED,
				MotiveStatus.ACTIVE, MOTIVE_ACTIVATED);

		// when places are created
		// m_verbals.verbaliseOnAddition(Place.class, NEW_PLACE_GENERATOR);

		// when places are explored
		// m_verbals.verbaliseOnStateTransition(Place.class,
		// PLACE_STATUS_ACCESSOR, PlaceStatus.PLACEHOLDER,
		// PlaceStatus.TRUEPLACE, PLACE_EXPLORED_GENERATOR);
		//
		// // when places are deleted (i.e. exploration could be carried out)
		// m_verbals.verbaliseOnDeletion(Place.class,
		// PLACE_EXPLORATION_FAILED_GENERATOR);

		// // when navigation is told to move the robot
		// m_verbals.verbaliseOnAddition(NavCommand.class, NAV_CMD_GENERATOR);

		// // when plan execution is triggered
		// m_verbals.verbaliseCannedTextOnAddition(PlanProxy.class,
		// "Starting plan execution.");

		// when AVS is triggered
//		m_verbals.verbaliseCannedTextOnAddition(AVSCommand.class,
//				"Having a look around");

		// when a recognition command is triggered�
		m_verbals.verbaliseOnAddition(DetectionCommand.class,
				DETECTION_COMMAND_GENERATOR);

		// when an object is added to the spatial model -> this is once per
		// object class in place
		m_verbals.verbaliseOnAddition(ObjectPlaceProperty.class,
				OBJECT_PROPERTY_GENERATOR);

		// when an object is recognised at all -> is this every positive
		// recognition result
		// m_verbals.verbaliseOnAddition(VisualObject.class,
		// VISUAL_OBJECT_GENERATOR);

//		m_verbals.verbaliseOnOverwrite(ComaRoom.class,
//				new RoomCategoryTextGenerator());

		m_verbals.verbaliseCannedTextOnAddition(GatewayPlaceProperty.class,
				"Ah ha. Looks like there is a door here");

	}

	public void runComponent() {
		m_verbals.verbaliseCannedText("go dora go go go");
	}

	// verbaliseCannedTextOnStateTransition

}
