package dora.test;

import static org.junit.Assert.*;

import java.util.HashMap;
import java.util.Map;

import org.junit.Test;

public class DoraTestRunnerTest {

	@Test
	public void testConfigureMapOfStringString() {

		DoraTestRunner runner = new DoraTestRunner();
		Map<String, String> config = new HashMap<String, String>();
		config.put(
				DoraTestRunner.WMCHECK_PREFIX + "test1",
				"sa=testsa;xpath=//*[@prop='test'];type=eu.cogx.beliefs.slice.PerceptBelief;qualifier=EXISTS");
		config.put(
				DoraTestRunner.WMCHECK_PREFIX + "test2",
				"sa=testsa;xpath=//*[@prop='test'];type=eu.cogx.beliefs.slice.GroundedBelief;qualifier=ALL");
		runner.configureWMContentMatchers(config);
		assertTrue(runner
				.getCheck("test1")
				.toString()
				.equals("castutils.castextensions.WMContentMatcher: sa=testsa;xpath=//*[@prop='test'];type=eu.cogx.beliefs.slice.PerceptBelief;qualifier=EXISTS"));
		assertTrue(runner
				.getCheck("test2")
				.toString()
				.equals("castutils.castextensions.WMContentMatcher: sa=testsa;xpath=//*[@prop='test'];type=eu.cogx.beliefs.slice.GroundedBelief;qualifier=ALL"));
	}

}
