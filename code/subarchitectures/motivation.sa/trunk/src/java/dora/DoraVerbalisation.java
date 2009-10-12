package dora;

import java.util.Map;

import motivation.slice.ExploreMotive;
import motivation.slice.Motive;
import motivation.slice.MotiveStatus;
import motivation.slice.PlanProxy;
import motivation.util.castextensions.Accessor;
import SpatialData.NavCommand;
import SpatialData.Place;
import SpatialData.PlaceStatus;
import cast.architecture.ManagedComponent;

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
			default:
				return "I'm doing something, but I don't want talk about it";
			}
		}
	};

	private static final TextGenerator<ExploreMotive> EXPLORE_MOTIVE_ACTIVATED = new TextGenerator<ExploreMotive>() {
		@Override
		public String toText(ExploreMotive _i) {
			return "Activated motive to explore hypothesis for Place " + _i.placeID;
		}
	};

	private static Accessor<Motive, MotiveStatus> MOTIVE_STATUS_ACCESSOR = new Accessor<Motive, MotiveStatus>() {
		@Override
		public MotiveStatus access(Motive _entry) {
			return _entry.status;
		}
	};

	private static Accessor<ExploreMotive, MotiveStatus> EXPLORE_MOTIVE_STATUS_ACCESSOR = new Accessor<ExploreMotive, MotiveStatus>() {
		@Override
		public MotiveStatus access(ExploreMotive _entry) {
			// TODO Why does this all not work with inheritance
			return MOTIVE_STATUS_ACCESSOR.access(_entry);
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
		m_verbals.verbaliseOnStateTransition(ExploreMotive.class,
				EXPLORE_MOTIVE_STATUS_ACCESSOR, MotiveStatus.SURFACED,
				MotiveStatus.ACTIVE, EXPLORE_MOTIVE_ACTIVATED);

		// when places are created
		m_verbals.verbaliseOnAddition(Place.class, NEW_PLACE_GENERATOR);

		// when places are explored
		m_verbals.verbaliseOnStateTransition(Place.class,
				PLACE_STATUS_ACCESSOR, PlaceStatus.PLACEHOLDER,
				PlaceStatus.TRUEPLACE, PLACE_EXPLORED_GENERATOR);

		// when places are deleted (i.e. exploration could be carried out)
		m_verbals.verbaliseOnDeletion(Place.class,
				PLACE_EXPLORATION_FAILED_GENERATOR);

		// when navigation is told to move the robot
		m_verbals.verbaliseOnAddition(NavCommand.class, NAV_CMD_GENERATOR);

		// when plan execution is triggered
		m_verbals.verbaliseCannedTextOnAddition(PlanProxy.class,
				"Starting plan execution.");

	}

	public void runComponent() {
		m_verbals.verbaliseCannedText("Is it the review yet?");
	}

	// verbaliseCannedTextOnStateTransition

}
