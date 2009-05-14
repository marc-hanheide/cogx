/**
 * 
 */
package cast.testing;

import java.util.Properties;

import cast.architecture.subarchitecture.SubarchitectureWorkingMemory;
import cast.cdl.WorkingMemoryChange;
import cast.core.CASTUtils;

/**
 * @author nah
 *
 */
public class TestWorkingMemory extends SubarchitectureWorkingMemory {

	private static final String ALLOW_CHANGES = "--allow-changes";
	boolean m_allowChanges;
	
	/**
	 * @param _id
	 */
	public TestWorkingMemory(String _id) {
		super(_id);
		setSendXarchChangeNotifications(true);
	}
	
	
	@Override
	public void configure(Properties _config) {
		super.configure(_config);

		if(_config.containsKey(ALLOW_CHANGES)) {
			m_allowChanges = Boolean.parseBoolean(_config.getProperty(ALLOW_CHANGES));
			log("allowing changes: " + m_allowChanges);
		}	
		else {
			m_allowChanges = true;
		}
	}
	
	@Override
	public void receivePushData(String _src, WorkingMemoryChange _data) {
		if(!m_allowChanges) {
			println("incorrectly received wm change: " + CASTUtils.toString(_data));
			System.exit(CASTTest.TEST_FAIL);
		}
		
		super.receivePushData(_src, _data);
	
	}
	
}
