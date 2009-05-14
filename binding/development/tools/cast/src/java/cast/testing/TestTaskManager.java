/**
 * 
 */
package cast.testing;

import java.util.HashMap;

import cast.architecture.subarchitecture.AlwaysPositiveTaskManager;
import cast.cdl.TaskResult;
import cast.cdl.testing.CAST_TEST_FAIL;

/**
 * 
 * A testing sub-class of {@link AlwaysPositiveTaskManager}. This accepts all
 * tasks, but expects to see completion results in numerical order.
 * 
 * @author nah
 * 
 */
public class TestTaskManager extends AlwaysPositiveTaskManager {
	
	static String TEST_TASK = "test-task";
	
	HashMap<String, Integer> m_counts;
	
	
	public static int taskNumber(String _taskID) {
		return Integer.parseInt(_taskID.split(":")[1]);
	}
	
	public static String numberedTask(String _src, int _taskNumber) {
		return _src + "-" + TEST_TASK + ":" + _taskNumber;
	}

	
	/**
	 * @param _id
	 */
	public TestTaskManager(String _id) {
		super(_id);
		m_counts = new HashMap<String, Integer>();
	}

	@Override
	protected void taskCompleted(String _src, TaskResult _data) {
		super.taskCompleted(_src, _data);
		log("TestTaskManager.taskCompleted(): " + _src + " " + _data.m_id);

		int number = taskNumber(_data.m_id);
		
		if(m_counts.containsKey(_src)) {
			if(number == (m_counts.get(_src) + 1)) {
				m_counts.put(_src, number);
			}
			else {
				System.exit(CAST_TEST_FAIL.value);
			}
		}
		else {
			m_counts.put(_src, number);
		}
		

	}

}
