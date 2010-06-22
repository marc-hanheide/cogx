package patrol;

import java.util.Map;

import motivation.slice.ExploreMotive;
import motivation.slice.HomingMotive;
import motivation.slice.Motive;
import motivation.slice.MotiveStatus;
import motivation.slice.PatrolMotive;
import motivation.util.castextensions.Accessor;
import SpatialData.Place;
import SpatialData.PlaceStatus;
import SpatialProperties.GatewayPlaceProperty;
import cast.architecture.ManagedComponent;
import dora.RandomStringBank;
import dora.TextGenerator;
import dora.VerbalisationFacade;

/**
 * 
 * @author nah
 * 
 */
public class PatrolDemoVerbalisation extends ManagedComponent {

	private static final RandomStringBank DOOR_SAYINGS = new RandomStringBank(
			new String[] { "Ah ha. Looks like there is a door here",
					"Are you a door?",
					"I think I have just gone through a door" });

	private static final RandomStringBank NEW_PLACE_SAYINGS = new RandomStringBank(
			new String[] { "Boink", "I will come back here", "Keep going",
					"Steady on", "Easy there driver", "", "", "", "",
					"Beep beep", "Coming through", "Excuse me please",
					"Look out", "I'll be back", "Danger",
					"Hello there handsome" });

	private static final RandomStringBank HOMING_SAYINGS = new RandomStringBank(
			new String[] { "I am going home",
					"home again, home again jiggedy jig", "bye bye" });

	private static final RandomStringBank PATROL_SAYINGS = new RandomStringBank(
			new String[] { "Over here now", "Off I go again", "This way",
					"Excuse me please", "Beep beep", "What is over here",
					"Time to go", "Look out", "Coming though", "Hello" });

	private static final RandomStringBank EXPLORE_SAYINGS = new RandomStringBank(
			new String[] { "What's over here",
					"Is there something new I should see?", "What else is new?" });
	//
	// private static final TextGenerator<NavCommand> NAV_CMD_GENERATOR = new
	// TextGenerator<NavCommand>() {
	// @Override
	// public String toText(NavCommand _i) {
	// switch (_i.cmd) {
	// case GOTOPLACE:
	// return "Going to Place " + _i.destId[0];
	// case TURNTO:
	// return "Turning";
	//
	// default:
	// // return "I'm doing something, but I don't want talk about it";
	// return "";
	// }
	// }
	// };

	private static final TextGenerator<Motive> MOTIVE_ACTIVATED = new TextGenerator<Motive>() {
		@Override
		public String toText(Motive _i) {

			if (_i instanceof ExploreMotive) {
				return EXPLORE_SAYINGS.next();
			} else if (_i instanceof PatrolMotive) {
				return PATROL_SAYINGS.next();
			} else if (_i instanceof HomingMotive) {
				return HOMING_SAYINGS.next();
			}
			return "";
		}
	};

	private static Accessor<Motive, MotiveStatus> MOTIVE_STATUS_ACCESSOR = new Accessor<Motive, MotiveStatus>() {
		@Override
		public MotiveStatus access(Motive _entry) {
			return _entry.status;
		}
	};

	private static TextGenerator<Place> NEW_PLACE_GENERATOR = new TextGenerator<Place>() {

		@Override
		public String toText(Place _i) {

			if (_i.status == PlaceStatus.PLACEHOLDER) {
				return "";
			} else {
				return NEW_PLACE_SAYINGS.next();
			}
		}
	};

	private final VerbalisationFacade m_verbals;

	public PatrolDemoVerbalisation() {
		m_verbals = new VerbalisationFacade(this);
	}

	@Override
	public void configure(Map<String, String> _config) {
		m_verbals.configure(_config);
	}

	public void start() {

		// when motives are activated
		m_verbals.verbaliseOnStateTransition(Motive.class,
				MOTIVE_STATUS_ACCESSOR, MotiveStatus.SURFACED,
				MotiveStatus.ACTIVE, MOTIVE_ACTIVATED);

		// when new true places are created
		m_verbals.verbaliseOnAddition(Place.class, NEW_PLACE_GENERATOR);

		// when a door node is created
		m_verbals.verbaliseCannedTextOnAddition(GatewayPlaceProperty.class,
				DOOR_SAYINGS);

	}

	public void runComponent() {
		m_verbals
				.verbaliseCannedText("Hello, my name is Dora and I will be your robot today.");

		// spout random sayings and random intervals
		while (isRunning()) {
			sleepComponent((long) (10000 + Math.random() * 20000));
			m_verbals.verbaliseCannedText(NEW_PLACE_SAYINGS.next());
		}

	}

}
