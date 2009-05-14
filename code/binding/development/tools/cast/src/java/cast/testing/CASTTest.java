/**
 * 
 */
package cast.testing;

import java.io.File;
import java.io.FileNotFoundException;

import cast.cdl.testing.CAST_TEST_FAIL;
import cast.cdl.testing.CAST_TEST_PASS;

/**
 * @author nah
 * 
 */
public class CASTTest {

	public static final long DEFAULT_TIMEOUT = 60000;

	public static final int TEST_PASS = CAST_TEST_PASS.value;

	public static final int TEST_FAIL = CAST_TEST_FAIL.value;

	private final String m_name;

	private final File m_castFile;

	private final int m_returnPass;

	private final long m_timeout;

	private final boolean m_output;

	public CASTTest(String _name, String _castFile)
			throws FileNotFoundException {
		this(_name, _castFile, TEST_PASS, DEFAULT_TIMEOUT, false);
	}

	public CASTTest(String _name, String _castFile, int _returnPass,
			long _timeout, boolean _output) throws FileNotFoundException {

		m_name = _name;
		m_castFile = new File(_castFile);
		if (!m_castFile.exists()) {
			throw new FileNotFoundException("CAST file does not exist: "
					+ m_castFile.getAbsolutePath());
		}

		m_returnPass = _returnPass;
		m_timeout = _timeout;
		m_output = _output;
	}

	public File getCASTFile() {
		return m_castFile;
	}

	public int getPassValue() {
		return m_returnPass;
	}

	public long getTimeout() {
		return m_timeout;
	}

	public String getName() {
		return m_name;
	}

	public boolean showOutput() {
		return m_output;
	}

}
