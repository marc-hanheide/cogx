/**
 * 
 */
package cast.testing;

import java.util.LinkedHashMap;
import java.util.LinkedList;
import java.util.Properties;
import java.util.Queue;

import cast.architecture.subarchitecture.PrivilegedManagedProcess;
import cast.cdl.WorkingMemoryChangeQueueBehaviour;
import cast.cdl.testing.CAST_TEST_FAIL;
import cast.cdl.testing.CAST_TEST_PASS;
import cast.core.CASTException;

/**
 * First pass class for generating test behaviour in an architecture.
 * 
 * @author nah
 * 
 */
public class AbstractTester extends PrivilegedManagedProcess {

	private LinkedHashMap<String, AbstractTest> m_tests;

	private Queue<String> m_perform;

	private boolean m_exitOnCompletion = true;

	private int m_successValue = CAST_TEST_PASS.value;

	private int m_failureValue = CAST_TEST_FAIL.value;

	private AbstractTest m_currentTest;

	private Thread m_testThread;

	/**
	 * Superclass of tests to be performed.
	 * 
	 * @author nah
	 * 
	 */
	public abstract class AbstractTest implements Runnable {

		private Object m_completionLock;

		private boolean m_testComplete;

		private boolean m_passed;


		public AbstractTest() {
			m_completionLock = new Object();
		}

		protected abstract void startTest();

		protected void stopTest() {

		}

		protected final void testComplete(boolean _passed) {
			m_testComplete = true;
			m_passed = _passed;
			synchronized (m_completionLock) {
				m_completionLock.notify();
			}
		}

		public void run() {
			m_testComplete = false;
			m_passed = false;
			lockProcess();
			startTest();
			unlockProcess();
		}

		public void stop() {
			println("stopping test");
			stopTest();
		}

		private boolean waitFor() {

			while (!m_testComplete) {
				synchronized (m_completionLock) {
					try {
						m_completionLock.wait();
					} catch (InterruptedException e) {
						e.printStackTrace();
					}
				}
			}

			return m_passed;

		}

	}

	/**
	 * @param _id
	 */
	public AbstractTester(String _id) {
		super(_id);
		m_queueBehaviour = WorkingMemoryChangeQueueBehaviour.QUEUE;
	}

	@Override
	public void configure(Properties _config) {
		super.configure(_config);

		if (_config.containsKey("--exit")) {
			m_exitOnCompletion = Boolean.parseBoolean(_config
					.getProperty("--exit"));
			log("m_exitOnCompletion = " + m_exitOnCompletion);
		}

		if (_config.containsKey("--test")) {
			String testString = _config.getProperty("--test");
			String[] tests = testString.split(",");
			for (String test : tests) {
				try {
					queueTest(test);
				} catch (CASTException e) {
					e.printStackTrace();
				}
			}
		}

	}
	
	/**
	 * Registers a new test to be performed when the given id is provided. Only
	 * the first test registered for an id is valid.
	 * 
	 * @param _id
	 *            The test id
	 * @param _test
	 *            The test object
	 * @throws CASTException
	 *             If test _id already exists
	 */
	protected void registerTest(String _id, AbstractTest _test)
			throws CASTException {

		assert _id != null : "id must be non null";
		assert _test != null : "test must be non null";
		assert _id.length() > 0 : "id must be non empty";

		if (m_tests == null) {
			m_tests = new LinkedHashMap<String, AbstractTest>();
		}

		if (m_tests.containsKey(_id)) {
			throw new CASTException("test already exists: " + _id);
		} else {
			m_tests.put(_id, _test);
		}

	}

	/**
	 * Queue the given test as behaviour to be performed when the component is
	 * run. If no tests are queue, then all registered tests are performed.
	 * 
	 * 
	 * @param _id
	 * @throws CASTException
	 */
	protected void queueTest(String _id) throws CASTException {

		assert _id != null : "id must be non null";
		assert _id.length() > 0 : "id must be non empty";

		if (m_tests.containsKey(_id)) {

			if (m_perform == null) {
				m_perform = new LinkedList<String>();
			}

			m_perform.add(_id);

		} else {
			throw new CASTException("unknown test to queue: " + _id);
		}

	}

	private boolean runTests() {
		boolean passed = true;
			while (!m_perform.isEmpty()) {
				// get mexy test to perform
				String nextTestID = m_perform.poll();
				// retrieve test and remove from map	
				log("start performing: " + nextTestID);
				AbstractTest nextTest = m_tests.get(nextTestID);
				String output = null;
				if (performTest(nextTest)) {
					output = ": passed";
				} else {
					output = ": failed";
					passed = false;
				}

				//get a lock on the process to ensure
				//nothing nasty happens to overlap
				//tests! -- Hendrik's problem ;)
				//				lockProcess();
				//unlockProcess();

				log(nextTestID + output);
		}
		return passed;
	}

	private boolean performTest(AbstractTest _nextTest) {

		assert _nextTest != null : "next test is null";

		m_currentTest = _nextTest;

		// create separate thread for running the test
		m_testThread = new Thread(_nextTest);

		// start that thread and the test
		m_testThread.start();

		// println("waiting for completion");

		// wait for the test fo complete
		boolean passed = m_currentTest.waitFor();

		m_testThread = null;
		m_currentTest = null;

		// println("complete");
		return passed;
	}

	@Override
	public void runComponent() {

		// a bunch of things that may go wrong
		assert (getProcessIdentifier() != null && !getProcessIdentifier()
				.equals(""));
		assert (m_subarchitectureID != null && !m_subarchitectureID.equals(""));
		assert m_tests != null : "no tests registered";

		// if no tests are explicitly queued, perform all registered tests
		if (m_perform == null) {
			log("no tests queued, performing all registered tests instead");
			m_perform = new LinkedList<String>(m_tests.keySet());
		}

		boolean allPassed = runTests();

		// check we're still running
		if (m_exitOnCompletion) {
			sleepProcess(5000);
			if (allPassed) {
				System.exit(m_successValue);
			} else {
				System.exit(m_failureValue);
			}
		}

	}

	/*
	 * (non-Javadoc)
	 * 
	 * @see cast.architecture.subarchitecture.ManagedProcess#taskAdopted(java.lang.String)
	 */
	@Override
	protected void taskAdopted(String _taskID) {

	}

	/*
	 * (non-Javadoc)
	 * 
	 * @see cast.architecture.subarchitecture.ManagedProcess#taskRejected(java.lang.String)
	 */
	@Override
	protected void taskRejected(String _taskID) {

	}

}
