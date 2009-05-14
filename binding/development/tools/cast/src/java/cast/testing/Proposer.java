/**
 * 
 */
package cast.testing;

import java.util.Properties;

import cast.architecture.subarchitecture.ManagedProcess;
import cast.architecture.subarchitecture.SubarchitectureProcessException;
import cast.cdl.TaskOutcome;
import cast.cdl.testing.CAST_TEST_FAIL;
import cast.cdl.testing.CAST_TEST_PASS;

/**
 * Test class for proposing m_count tasks and counting and checking the ordering
 * of the responses.
 * 
 * @author nah
 * 
 */
public class Proposer extends ManagedProcess {

	private static enum ExpectedResponse {
		ADOPTED, REJECTED, MIXED
	};

	private int m_count;

	private int m_expected;

	private ExpectedResponse m_response;

	// private long m_sleep

	/**
	 * @param _id
	 */
	public Proposer(String _id) {
		super(_id);
		m_count = 0;
		m_expected = 0;
	}

	@Override
	protected void runComponent() {

		for (int i = 0; i < m_count; i++) {
			lockProcess();
			try {
				proposeInformationProcessingTask(TestTaskManager.numberedTask(
						getProcessIdentifier(), i), TestTaskManager.TEST_TASK);
				randomSleep();
			} catch (SubarchitectureProcessException e) {
				e.printStackTrace();
			}
			unlockProcess();
		}

	}

	@Override
	public void configure(Properties _config) {
		super.configure(_config);

		String count = _config.getProperty("--count");
		if (count != null) {
			m_count = Integer.parseInt(count);
		}

		String response = _config.getProperty("--response");
		if (response != null) {
			m_response = ExpectedResponse.valueOf(response);
		} else {
			m_response = ExpectedResponse.ADOPTED;
		}

	}

	/*
	 * (non-Javadoc)
	 * 
	 * @see cast.architecture.subarchitecture.ManagedProcess#taskAdopted(java.lang.String)
	 */
	@Override
	protected void taskAdopted(String _taskID) {
		log("taskAdopted: " + _taskID);
		if (m_response == ExpectedResponse.ADOPTED
				|| m_response == ExpectedResponse.MIXED) {

			int number = TestTaskManager.taskNumber(_taskID);
			if (number == m_expected) {
				m_expected++;

				randomSleep();

				// now say we're done
				try {
					taskComplete(_taskID,
							TaskOutcome.PROCESSING_COMPLETE_SUCCESS);
				} catch (SubarchitectureProcessException e) {
					// TODO Auto-generated catch block
					e.printStackTrace();
				}

				if (m_expected == m_count) {
					// sleep a little to let everyone else finis
					sleepProcess(5000);
					System.exit(CAST_TEST_PASS.value);
				}
			} else {
				System.exit(CAST_TEST_FAIL.value);
			}

		} else {
			System.exit(CAST_TEST_FAIL.value);
		}

	}

	private void randomSleep() {
		try {
			Thread.sleep((long) (Math.random() * 500));
		} catch (InterruptedException e) {
			e.printStackTrace();
		}
	}

	/*
	 * (non-Javadoc)
	 * 
	 * @see cast.architecture.subarchitecture.ManagedProcess#taskRejected(java.lang.String)
	 */
	@Override
	protected void taskRejected(String _taskID) {
		log("taskRejected: " + _taskID);
		if (m_response == ExpectedResponse.REJECTED
				|| m_response == ExpectedResponse.MIXED) {

			int number = TestTaskManager.taskNumber(_taskID);
			if (number == m_expected) {
				m_expected++;

				if (m_expected == m_count) {
					// sleep a little to let everyone else finis
					sleepProcess(5000);
					System.exit(CAST_TEST_PASS.value);
				}
			} else {
				System.exit(CAST_TEST_FAIL.value);
			}

		} else {
			System.exit(CAST_TEST_FAIL.value);
		}
	}

}
