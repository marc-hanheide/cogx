package tts;

import java.util.ArrayList;
import java.util.Map;

import cast.cdl.WorkingMemoryOperation;
import castutils.components.CommandLineOnOperation;
import de.dfki.lt.tr.dialogue.slice.synthesize.SpokenOutputItem;

/**
 * 
 * Simple command-line TTS component for systems which can't run Mary. Can be
 * used with espeak (linux, osx), say (osx) and any other tool that gnerates
 * speech from text on the command line.
 * 
 * Should be configured with comma-separated list of arguments which will be
 * combined to make the tts command. E.g.
 * 
 * --cmd-prefix "say,-v,Alex"
 * 
 * will result in the command
 * 
 * say -v Alex {@link SpokenOutputItem}.phonString.
 * 
 * 
 * 
 * 
 * @author nah
 * 
 */
public class CommandLineTTS extends CommandLineOnOperation<SpokenOutputItem> {

	private ArrayList<String> m_commandPrefix;
	private boolean m_test = false;

	public static final String COMMAND_PREFIX_KEY = "--cmd-prefix";
	public static final String TEST_KEY = "--test";

	public CommandLineTTS() {
		super(WorkingMemoryOperation.ADD, SpokenOutputItem.class);
		m_commandPrefix = new ArrayList<String>();
		m_commandPrefix.add("say");
		// add a placeholder at the end to overwrite with actual output data
		m_commandPrefix.add("nothing");
	}

	@Override
	protected void configure(Map<String, String> _config) {
		String prefix = _config.get(COMMAND_PREFIX_KEY);
		if (prefix != null) {
			String[] commandPrefix = prefix.split(",");
			assert (commandPrefix.length > 0);
			m_commandPrefix.clear();
			for (String s : commandPrefix) {
				m_commandPrefix.add(s);
			}
			// add a placeholder at the end to overwrite with actual output data
			m_commandPrefix.add("nothing");
		}
		log("using command prefix: " + m_commandPrefix);

		if (_config.containsKey(TEST_KEY)) {
			m_test = true;
		}
	}

	@Override
	protected ProcessBuilder getProcess(SpokenOutputItem _cmd) {
		m_commandPrefix.set(m_commandPrefix.size() - 1, _cmd.phonString);
		return new ProcessBuilder(m_commandPrefix);
	}

	@Override
	protected void runComponent() {
		if (m_test) {
			SpokenOutputItem out = new SpokenOutputItem();
			out.phonString = "one two three four";
			try {
				executeProcess(getProcess(out));
			} catch (Exception e) {
				logException(e);
			}
		}
	}

}
