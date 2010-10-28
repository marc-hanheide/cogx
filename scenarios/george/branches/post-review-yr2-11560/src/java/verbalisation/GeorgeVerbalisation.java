package verbalisation;

import java.util.HashSet;
import java.util.Map;
import java.util.Set;

import motivation.slice.CannedTextMotive;
import motivation.slice.MotiveStatus;
import cast.architecture.ManagedComponent;
import cast.cdl.WorkingMemoryAddress;

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
public class GeorgeVerbalisation extends ManagedComponent {

	private final TextGenerator<CannedTextMotive> CANNED_CMD_GENERATOR = new TextGenerator<CannedTextMotive>() {

		final Set<WorkingMemoryAddress> uttered = new HashSet<WorkingMemoryAddress>();

		@Override
		public String toText(CannedTextMotive _i) {
			if (uttered.contains(_i.thisEntry))
				return "";
			if (_i.status == MotiveStatus.SURFACED) {
				uttered.add(_i.thisEntry);
				return _i.text;

			} else
				return "";
		}
	};

	private final VerbalisationFacade m_verbals;

	public GeorgeVerbalisation() {
		m_verbals = new VerbalisationFacade(this);
	}

	@Override
	public void configure(Map<String, String> _config) {
		m_verbals.configure(_config);
	}

	@Override
	public void start() {

		// say stuff...

		m_verbals.verbaliseOnOverwrite(CannedTextMotive.class,
				CANNED_CMD_GENERATOR);

	}

	@Override
	public void runComponent() {
		m_verbals.verbaliseCannedText("my name is george.");
	}

}
