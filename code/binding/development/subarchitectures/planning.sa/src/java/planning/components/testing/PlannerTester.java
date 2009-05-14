package planning.components.testing;

import java.util.Properties;

import planning.util.MAPSIMAgent;
import planning.util.PlannerServerFactory;

import cast.core.CASTException;
import cast.testing.AbstractTester;

public class PlannerTester extends AbstractTester {

	public PlannerTester(String _id) {
		super(_id);
	}

	
	private class ServerInterfaceTest extends AbstractTest {
		@Override
		protected void startTest() {
			//ok, it's a bit cheeky, but hey
			testComplete(PlannerServerFactory.testServer());
		}
	}

	private class ServerSelfTest extends AbstractTest {
		@Override
		protected void startTest() {
			//ok, it's a bit cheeky, but hey
			testComplete(PlannerServerFactory.selfTestServer());
		}
	}
	
	private class AgentTest extends AbstractTest {
		@Override
		protected void startTest() {
			//ok, it's a bit cheeky, but hey
			testComplete(MAPSIMAgent.testMAPSIM());
		}
	}
	@Override
	public void configure(Properties _config) {

		try {
			registerTest("self-test", new ServerSelfTest());
			registerTest("interface-test", new ServerInterfaceTest());
			registerTest("agent-test", new AgentTest());
		}
		catch (CASTException e) {
			println(e);
		}
	
		super.configure(_config);
	}
	
}
