package planning.util;

public class MAPSIMAgentFactory {

	private static final String PLAYMATE_AGENT_NAME = "MrChips";
	private static final String PLAYMATE_DOMAIN_NAME = "playmate";
	private static final String PLAYMATE_DOMAIN_FILE = "subarchitectures/planning.sa/src/python/mapsim/domains/playmate/mapl_files/domain.mapl";

	public static MAPSIMAgent newPlayMateAgent(boolean _log) {
		return newMAPSIMAgent(PLAYMATE_DOMAIN_NAME, PLAYMATE_DOMAIN_FILE,
				PLAYMATE_AGENT_NAME, _log);
	}

	public static MAPSIMAgent newMAPSIMAgent(String _domain,
			String _domainFile, String _agentName, boolean _log) {
		return new MAPSIMAgent(_domain, _domainFile, _agentName, _log);
	}
}
