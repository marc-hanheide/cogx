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

	private static final String WAIT_KEY = "--wait";
	public static final String WMCHECK_PREFIX = "--wmcheck-";
	Map<String, WMContentMatcher<?>> checks = new HashMap<String, WMContentMatcher<?>>();
	private int waitTime = 0;

	@Override
	public MotiveStatus submitGoal(String goalString, float importance,
			Current __current) {
		MotiveStatus goalExecutionStatus = super.submitGoal(goalString,
				importance, __current);
		if (goalExecutionStatus == MotiveStatus.COMPLETED) {
			sleepComponent(waitTime * 1000);
			boolean memCheck = verifyMemoryContent();
			if (!memCheck)
				return MotiveStatus.IMPOSSIBLE;
			else
				return goalExecutionStatus;
		} else
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
		String waitTimeStr = config.get(WAIT_KEY);
		if (waitTimeStr != null) {
			waitTime = Integer.parseInt(waitTimeStr);
		}

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
					check.setPrintMatches(true);
					checks.put(checkName, check);
					println("added check " + checkName + ": " + check);
				} catch (ClassNotFoundException e) {
					logException(e);
				}
			}
		}
	}
}
