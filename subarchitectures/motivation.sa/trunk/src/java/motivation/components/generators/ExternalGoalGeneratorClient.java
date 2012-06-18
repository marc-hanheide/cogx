/**
 * 
 */
package motivation.components.generators;

import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;

import java.util.HashMap;
import java.util.Map;
import java.util.StringTokenizer;

import motivation.slice.ExternalGoalServer;
import motivation.slice.ExternalGoalServerPrx;
import motivation.slice.MotiveStatus;

import org.apache.log4j.Logger;
import org.junit.Test;
import org.junit.internal.runners.JUnit38ClassRunner;

import cast.CASTException;
import cast.cdl.ComponentLanguage;
import cast.configuration.ArchitectureConfigurationException;
import cast.core.CASTUtils;
import castutils.castextensions.IceXMLSerializer;

/**
 * implements a JUnit4 test where it calls the ExternalGoalGenerator component
 * with a predefined goal string that will then cause a GeneralGoalMotive to be
 * created and monitored in the working memory.
 * 
 * Here is an example how this can be invoked in an ANT file:
 * 
 * <pre>
 * 	<target name="goaltest">
 * 		<junit fork="no">
 * 			<sysproperty key="goal" value="${test.goal}"/>
 * 		  	<classpath>
 * 				<fileset dir="${jar.dir}">
 *   					<include name="*.jar"/>
 * 				</fileset>
 *     			<pathelement location="${output.dir}"/>
 *     			<pathelement location="${cast.jar}"/>
 *     			<pathelement location="${ice.jar}"/>
 *   			</classpath>
 * 
 *     		<formatter type="xml"/> 
 *   			
 * 			<test name="motivation.components.generators.ExternalGoalGeneratorClient" haltonfailure="yes" outfile="result" />
 * 		</junit>
 * 	</target>
 * 
 * </pre>
 * 
 * @author marc
 * 
 */
/** talks to the {@link ExternalGoalGenerator}
 * @author marc
 * the Junit test should be configured using system property "config", e.g. as follows:
 * <code>--goal "(and )" --importance -1 --wmcheck-test1 "sa=testsa;xpath=//*[@prop='test'];type=eu.cogx.beliefs.slice.PerceptBelief;qualifier=EXISTS"</code>
 */
public class ExternalGoalGeneratorClient {

	private static final String SYSPROP_SERVER_NAME = "server-name";
	private static final String SYSPROP_HOSTNAME = "hostname";
	private static final String SYSPROP_CONFIG = "config";
	private static final String KEY_IMPORTANCE = "importance";
	private static final String KEY_GOAL = "goal";
	/** timeout of 30 minutes */
	private static final long TIMEOUT = 30 * 60 * 1000;
	static Logger logger = Logger.getLogger(ExternalGoalGeneratorClient.class);
	private String serverHost;
	private String serverName;
	private ExternalGoalServerPrx server;
	private Map<String, String> config;
	public static final String COMMENT_CHAR = "#";
	private static final String TRUE_VALUE = "true";

	void init() throws CASTException {
		logger.debug("creating server proxy");
		try {
			server = CASTUtils.getCASTIceServer(serverName, serverHost,
					ComponentLanguage.CPP, ExternalGoalServer.class,
					ExternalGoalServerPrx.class);
		} catch (CASTException e) {
			server = CASTUtils.getCASTIceServer(serverName, serverHost,
					ComponentLanguage.JAVA, ExternalGoalServer.class,
					ExternalGoalServerPrx.class);
		}
	}

	boolean call(String goalString, float importance) {
		logger.info("configuring the server");
		server.config(config);
		logger.info("calling the server with goal " + goalString);
		MotiveStatus result = server.submitGoal(goalString, importance);
		logger.info("server returned with " + result);
		if (result == MotiveStatus.ACTIVE || result == MotiveStatus.COMPLETED)
			return true;
		else
			return false;
	}

	String getGoalFromProperty() {
		String g = System.getProperty(KEY_GOAL);

		if (g == null)
			if (config.containsKey("--"+KEY_GOAL)) {
				g = config.get("--"+KEY_GOAL);
			} else
				g = "(and )";
		return g;
	}

	float getImportanceFromProperty() {
		String i = System.getProperty(KEY_IMPORTANCE);
		if (i == null)
			if (config.containsKey("--"+KEY_IMPORTANCE)) {
				i = config.get("--"+KEY_IMPORTANCE);
			} else
				i = "-1";
		return Float.parseFloat(i);
	}

	String getConfigFromProperty() {
		String i = System.getProperty(SYSPROP_CONFIG);
		if (i == null)
			i = "";
		return i;
	}

	@Test(timeout = TIMEOUT)
	public void testGoal() {
		try {
			config = parseCommandLine(getConfigFromProperty());
			String goal = getGoalFromProperty();
			float importance = getImportanceFromProperty();
			logger.info("got goal " + goal + ", got importance " + importance);
			init();
			assertTrue("the goal " + goal
					+ " has not been achieved, but has now been removed.",
					call(goal, importance));
		} catch (CASTException e) {
			logger.error("exception in testGoal", e);
			fail("test failed from exception: " + e.message);
		}
	}

	public ExternalGoalGeneratorClient() {
		super();
		serverHost = System.getProperty(SYSPROP_HOSTNAME);
		if (serverHost == null)
			serverHost = "localhost";
		serverName = System.getProperty(SYSPROP_SERVER_NAME);
		if (serverName == null)
			serverName = "goal.server";
	}

	protected static Map<String, String> parseCommandLine(String confLine)
			throws ArchitectureConfigurationException {
		logger.info("confLine=" + confLine);
		Map<String, String> prop = new HashMap<String, String>();
		StringTokenizer _tokeniser = new StringTokenizer(confLine);
		String token;
		String key = null;
		String value = null;
		while (_tokeniser.hasMoreTokens()) {
			token = _tokeniser.nextToken();

			if (token.startsWith(COMMENT_CHAR)) {
				if (key != null) {
					prop.put(key, TRUE_VALUE);
				}
				break;
			}
			// if it's a command flag
			else if (token.startsWith("-")) {
				// if we have a previous key, assume its a single arg
				if (key != null) {
					prop.put(key, TRUE_VALUE);
				}

				// else store the key
				key = token;

			} else {
				// we have a value

				// if it's a string
				if (token.startsWith("\"") && (!token.endsWith("\""))) {
					// collect bits of string into one
					value = token;
					while (_tokeniser.hasMoreTokens()) {
						token = _tokeniser.nextToken();

						// last bit of string
						if (token.endsWith("\"")) {
							value += " " + token;

							// strip off leading and trailing "s
							value = value.substring(1, value.length() - 1);

							prop.put(key, value);
							key = null;
							break;
						}
						// contents of string
						else {
							value += " " + token;
						}
					}

				} else {
					if (key != null) {
						// if the token looks like "fsdfd"
						if (token.startsWith("\"") && token.endsWith("\"")) {
							token = token.substring(1, token.length() - 1);
						}

						prop.put(key, token);
						key = null;
					} else {
						throw new ArchitectureConfigurationException(
								"Malformed command line args. Stray value: "
										+ token + " in '" + confLine + "'");
					}
				}

			}

		}

		// if we have a key here, it has to be no-valued
		if (key != null) {
			prop.put(key, TRUE_VALUE);
		}

		logger.info("configuration read: " + IceXMLSerializer.toXMLString(prop));

		return prop;
	}

	/**
	 * @param args
	 */
	public static void main(String[] args) {
		if (args.length < 1) {
			System.err
					.println("command line parameters: <goal> [<importance>]");
			System.exit(1);
		}

		String concatLine = "";
		for (String s : args) {
			if (s.contains(" ")) {
				s = "\"" + s + "\"";
			}
			concatLine += s + " ";
		}
		try {
			parseCommandLine(concatLine);
		} catch (ArchitectureConfigurationException e1) {
			// TODO Auto-generated catch block
			e1.printStackTrace();
		}

		ExternalGoalGeneratorClient client = new ExternalGoalGeneratorClient();
		try {
			client.init();
		} catch (CASTException e) {
			logger.error("exception in testGoal", e);
			System.exit(1);
		}

		String goal = args[0];
		float importance = -1;
		if (args.length > 1)
			importance = Float.parseFloat(args[1]);
		logger.info("got goal " + goal + ", got importance " + importance);
		if (client.call(goal, importance)) {
			logger.info("goal " + goal + " has been achieved.");
			System.exit(0);
		} else {
			logger.warn("goal " + goal + " has failed.");
			System.exit(1);
		}
	}

}
