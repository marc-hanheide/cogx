package tts;

import java.util.Map;

import Ice.Object;
import cast.cdl.WorkingMemoryOperation;

import comsys.datastructs.comsysEssentials.SpokenOutputItem;

public class CommandLineTTS extends CommandLineOnOperation<SpokenOutputItem> {

	private String m_commandPrefix = "espeak ";
	
	public static final String COMMAND_PREFIX_KEY = "--cmd-prefix";
	
	public CommandLineTTS() {
		super(WorkingMemoryOperation.ADD, SpokenOutputItem.class);
		// TODO Auto-generated constructor stub
	}

	@Override
	protected void configure(Map<String, String> _config) {
		String prefix = _config.get(COMMAND_PREFIX_KEY);
		if(prefix != null) {
			m_commandPrefix = prefix;
			m_commandPrefix += ' ';
			
		}
		log("using command prefix: " + m_commandPrefix);
	}
	
	@Override
	protected String getCommand(SpokenOutputItem _cmd) {
		assert(_cmd != null);
		return m_commandPrefix + _cmd.phonString;
	}

}
