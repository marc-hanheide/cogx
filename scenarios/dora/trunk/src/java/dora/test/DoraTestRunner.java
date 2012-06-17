package dora.test;

import java.util.HashMap;
import java.util.Map;
import java.util.Map.Entry;

import motivation.components.generators.ExternalGoalGenerator;
import motivation.slice.MotiveStatus;
import Ice.Current;
import Ice.Object;
import castutils.castextensions.WMContentMatcher;

public class DoraTestRunner extends ExternalGoalGenerator {

	public static final String WMCHECK_PREFIX = "--wmcheck-";
	Map<String, WMContentMatcher<?>> checks = new HashMap<String, WMContentMatcher<?>>();

	@Override
	public MotiveStatus submitGoal(String goalString, float importance,
			Current __current) {
		MotiveStatus goalExecutionStatus = super.submitGoal(goalString,
				importance, __current);

		boolean memCheck = verifyMemoryContent();
		if (!memCheck)
			return MotiveStatus.IMPOSSIBLE;
		else
			return goalExecutionStatus;
	}

	public WMContentMatcher<?> getCheck(String key) {
		return checks.get(key);
	}

	private boolean verifyMemoryContent() {
		for (Entry<String, WMContentMatcher<?>> check : checks.entrySet()) {
			println("run check " + check.getKey() + ": " + check.getValue());
			if (!check.getValue().check())
				return false;
		}
		return true;
	}

	@Override
	protected void configure(Map<String, String> config) {
		super.configure(config);
		configureWMContentMatchers(config);

	}

	/**
	 * @param config
	 */
	protected void configureWMContentMatchers(Map<String, String> config) {
		for (String key : config.keySet()) {
			if (key.startsWith(WMCHECK_PREFIX)) {
				String checkName = key.substring(WMCHECK_PREFIX.length());
				try {
					WMContentMatcher<Object> check = WMContentMatcher.create(
							this, config.get(key));
					checks.put(checkName, check);
					println("added check " + checkName + ": " + check);
				} catch (ClassNotFoundException e) {
					logException(e);
				}
			}
		}
	}
}
