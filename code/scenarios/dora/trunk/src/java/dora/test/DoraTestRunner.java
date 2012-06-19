package dora.test;

import java.util.HashMap;
import java.util.Map;
import java.util.Map.Entry;

import motivation.components.generators.ExternalGoalGenerator;
import motivation.slice.MotiveStatus;
import Ice.Current;
import Ice.Object;
import castutils.castextensions.WMContentInjector;
import castutils.castextensions.WMContentMatcher;

public class DoraTestRunner extends ExternalGoalGenerator {

	private static final String WAIT_KEY = "--wait";

	public static final String WMCHECK_PREFIX = "--wmcheck-";

	Map<String, WMContentMatcher<?>> checks = new HashMap<String, WMContentMatcher<?>>();
	private WMContentInjector injector;
	private String POST_GOAL_INJECTION_KEY = "--inject-after-goal";
	private String PRE_GOAL_INJECTION_KEY = "--inject-before-goal";
	private int waitTime = 0;

	private String preGoalInjection;

	private String postGoalInjection;

	public DoraTestRunner() {
		super();
		injector = new WMContentInjector(this);
	}

	@Override
	protected void configure(Map<String, String> config) {
		super.configure(config);
		configureWMContentMatchers(config);
		String waitTimeStr = config.get(WAIT_KEY);
		if (waitTimeStr != null) {
			waitTime = Integer.parseInt(waitTimeStr);
		}

		preGoalInjection = config.get(PRE_GOAL_INJECTION_KEY);
		postGoalInjection = config.get(POST_GOAL_INJECTION_KEY);

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

	public WMContentMatcher<?> getCheck(String key) {
		return checks.get(key);
	}

	@Override
	public MotiveStatus submitGoal(String goalString, float importance,
			Current __current) {
		if (preGoalInjection != null)
			injector.inject(preGoalInjection);
		MotiveStatus goalExecutionStatus = super.submitGoal(goalString,
				importance, __current);
		if (goalExecutionStatus == MotiveStatus.COMPLETED) {
			if (postGoalInjection != null)
				injector.inject(postGoalInjection);
			sleepComponent(waitTime * 1000);
			boolean memCheck = verifyMemoryContent();
			if (!memCheck)
				return MotiveStatus.IMPOSSIBLE;
			else
				return goalExecutionStatus;
		} else
			return goalExecutionStatus;
	}

	private boolean verifyMemoryContent() {
		for (Entry<String, WMContentMatcher<?>> check : checks.entrySet()) {
			println("run check " + check.getKey() + ": " + check.getValue());
			if (!check.getValue().check())
				return false;
		}
		return true;
	}
}
