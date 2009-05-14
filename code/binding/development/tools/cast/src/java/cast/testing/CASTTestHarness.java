/**
 * 
 */
package cast.testing;

import java.io.BufferedReader;
import java.io.BufferedWriter;
import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileReader;
import java.io.FileWriter;
import java.io.IOException;
import java.io.InputStream;
import java.io.InputStreamReader;
import java.util.ArrayList;
import java.util.List;
import java.util.Vector;

import org.w3c.dom.Document;
import org.w3c.dom.Node;
import org.w3c.dom.NodeList;
import org.xml.sax.InputSource;
import org.xml.sax.SAXException;

import balt.jni.NativeProcessLauncher;

import com.sun.org.apache.xerces.internal.parsers.DOMParser;

/**
 * @author nah
 * 
 */
public class CASTTestHarness {

	/***************************************************************************
	 * time to wait between checking for process termination if a timeout is
	 * used
	 **************************************************************************/
	private long m_cycleTimeMillis = 1000;

	private enum TestResult {
		// the test was passed
		PASS_TEST,
		// the test was failed
		FAIL_TEST,
		// the test failed to run
		FAIL_RUN
	};

	ArrayList<CASTTest> m_tests;

	private Thread m_outputThread;

	private final String m_namingHost;

	private OutputRunnable m_outputRunnable;
    
        private boolean m_noPID;
    
        public CASTTestHarness(String _namingHost) {
		m_tests = new ArrayList<CASTTest>();
		m_namingHost = _namingHost;
                m_noPID = false;
	}

	public void addTest(CASTTest _test) {
		m_tests.add(_test);
	}
    
        public void disablePID() {
	    m_noPID = true;
	}

	/**
	 * @param args
	 * @throws FileNotFoundException
	 */
	public static void main(String[] args) throws FileNotFoundException {
		String namingHost = null;
		ArrayList<String> testFiles = new ArrayList<String>();
		boolean outputAll = false;
		boolean noPID = false;
		try {
			for (int i = 0; i < args.length; i++) {
				if (args[i].equals("-f")) {
					testFiles.add(args[i + 1]);
				} else if (args[i].equals("-h")) {
					namingHost = args[i + 1];
				} else if (args[i].equals("-o")) {
					outputAll = true;
				} else if (args[i].equals("--noPID")) {
				    noPID = true;
				}
			}
		} catch (ArrayIndexOutOfBoundsException e) {
		    showArgs();
		    return;
		}

		if (namingHost == null || testFiles.isEmpty()) {
			showArgs();
			return;
		}

		CASTTestHarness ctn = new CASTTestHarness(namingHost);
		// ctn.addTest(new CASTTest("tools/cast/config/single-write-mixed.cast",
		// CASTTest.TEST_PASS, 10000));
		// ctn.addTest(new CASTTest("tools/cast/config/test.cast"));
		// ctn.addTest(new CASTTest("tools/cast/config/test.cast",
		// CASTTest.TEST_PASS, 2000));
		if(noPID) {
		    ctn.disablePID();
		}
		System.out.println("pid: " + NativeProcessLauncher.getProcessID());
		for (String testFile : testFiles) {
			ctn.createTests(testFile, outputAll);
		}
		ctn.runTests();
	}

	private void createTests(String _file, boolean _outputAll)
			throws FileNotFoundException {

		DOMParser parser = new DOMParser();
		File f = new File(_file);
		FileReader reader = new FileReader(f);

		InputSource source = new InputSource(reader);
		try {

			parser.parse(source);
			Document testDoc = parser.getDocument();

			NodeList childNodes = testDoc.getChildNodes().item(0)
					.getChildNodes();

			for (int i = 0; i < childNodes.getLength(); i++) {
				Node testNode = childNodes.item(i);
				// System.out.println(childNodes.item(i).getLocalName());
				if (testNode.getNodeName().equals("test")) {
					CASTTest test = createTest(testNode, _outputAll);
					// System.out.println("created test: " + test.getName());
					addTest(test);
				}
			}

		} catch (SAXException e) {
			e.printStackTrace();
		} catch (IOException e) {
			e.printStackTrace();
		}

		System.out.println("test file: " + _file);

	}

	private CASTTest createTest(Node _testNode, boolean _output)
			throws FileNotFoundException {

		String file = null;
		String name = null;
		Long timeout = null;
		Integer passValue = null;
		Boolean output = _output;

		NodeList parameterNodes = _testNode.getChildNodes();
		for (int i = 0; i < parameterNodes.getLength(); i++) {
			Node parameter = parameterNodes.item(i);
			// ugly but fast
			if (parameter.getNodeName().equals("name")) {
				name = parameter.getFirstChild().getNodeValue();
			} else if (parameter.getNodeName().equals("file")) {
				file = parameter.getFirstChild().getNodeValue();
			} else if (parameter.getNodeName().equals("description")) {
				// ignored in code
			} else if (parameter.getNodeName().equals("timeout")) {
				timeout = Long.parseLong(parameter.getFirstChild()
						.getNodeValue());
			} else if (parameter.getNodeName().equals("output")) {
				output = Boolean.parseBoolean(parameter.getFirstChild()
						.getNodeValue());
			} else if (parameter.getNodeName().equals("pass")) {
				passValue = Integer.parseInt(parameter.getFirstChild()
						.getNodeValue());
			} else {
				assert true : "no other nodes allowed";
			}
		}

		// enforce required fields
		// TODO validation!
		assert (name != null && file != null);

		// fill in defaults
		if (timeout == null) {
			timeout = CASTTest.DEFAULT_TIMEOUT;
		}
		if (output == null) {
			output = false;
		}
		if (passValue == null) {
			passValue = CASTTest.TEST_PASS;
		}

		return new CASTTest(name, file, passValue, timeout, output);
	}

	private static void showArgs() {
		System.err
				.println("CASTTestHarness arguments:\n\t -h naming host REQUIRED\n\t -f test file REQUIRED\n\t -o (show output) OPTIONAL");

	}

	public void runTests() {
		int passes = 0;
		int fails = 0;

		for (CASTTest test : m_tests) {

			System.out.print(test.getName() + ": ");
			TestResult result = runTest(test);
			if (result == TestResult.PASS_TEST) {
				System.out.println("\033[32mpassed\033[0m");
				passes++;
			} else if (result == TestResult.FAIL_TEST) {
				System.out.println("\033[31m***FAILED***\033[0m");
				fails++;
			} else if (result == TestResult.FAIL_RUN) {
				System.out.println("unable to run");
			}

			// sleep for a little as the speed of restart may screw tnameserv up
			// a bit
			try {
				Thread.sleep(3000);
			} catch (InterruptedException e) {
				e.printStackTrace();
			}

		}

		int ran = passes + fails;
		System.out.println("result: " + passes + "/" + ran + ", "
				+ (m_tests.size() - ran) + " not runnable");
	}

	private static class OutputRunnable implements Runnable {
		private final InputStream m_stream;

		private final String m_prefix;

		private final List<String> m_buffer;

		private final boolean m_showOutput;

		public OutputRunnable(InputStream _stream, boolean _showOutput) {
			this(_stream, "test> ", _showOutput);
		}

		public OutputRunnable(InputStream _stream, String _prefix,
				boolean _showOutput) {
			m_stream = _stream;
			m_prefix = _prefix;
			// use a vector in case of threaded access
			m_buffer = new Vector<String>();
			m_showOutput = _showOutput;
		}

		public void run() {
			BufferedReader br = new BufferedReader(new InputStreamReader(
					m_stream));
			String s;
			try {
				if (m_showOutput) {
					while ((s = br.readLine()) != null) {
						System.out.println(m_prefix + s);
						m_buffer.add(s);
					}
				} else {
					while ((s = br.readLine()) != null) {
						m_buffer.add(s);
					}
				}
			} catch (IOException e) {
				e.printStackTrace();
			}
		}

		public List<String> getBuffer() {
			return m_buffer;
		}

	}

	private TestResult runTest(CASTTest _test) {

		String classpath = System.getenv("TEST_CLASSPATH");

		if (classpath == null) {
		    classpath = System.getProperty ("java.class.path");
//		    System.out.println("using classpath: " + classpath);
		}

		// System.out.println("using test classpath: " + classpath);

		ProcessBuilder pb = new ProcessBuilder("java", "-Xmx512M", "-ea",
				"-classpath", classpath, "cast.server.CASTProcessServer", "-h",
				m_namingHost, "-f", _test.getCASTFile().getAbsolutePath());
		try {
			pb.redirectErrorStream(true);
			Process p = pb.start();

			boolean showOutput = _test.showOutput();
			startOutput(p, showOutput);

			int exitCode = 0;

			long timeout = _test.getTimeout();
			long sleptFor = 0;

			// if we are waiting for a timeout
			if (timeout > 0) {

				boolean complete = false;

				while (!complete) {
					//
					try {
						// thi
						exitCode = p.exitValue();
						complete = true;
					} catch (IllegalThreadStateException e) {

						// if we've slept for longer than the test allows, kill
						// process
						if (sleptFor > timeout) {
							p.destroy();
							p.getInputStream().close();

							System.out.print((sleptFor/1000) +  " secs (time-out) ");
							stopOutput();
							dumpOutput(_test);

							return TestResult.FAIL_TEST;
						}
						// if process has not finished, sleep for a bit
						else {
							Thread.sleep(m_cycleTimeMillis);
							sleptFor += m_cycleTimeMillis;
						}

					}
				}

			}
			// else we're waiting
			else {
				exitCode = p.waitFor();
			}

			stopOutput();
			System.out.print((sleptFor/1000) +  " secs, ");
			if (_test.getPassValue() == exitCode) {
				return TestResult.PASS_TEST;
			} else {
				dumpOutput(_test);
				return TestResult.FAIL_TEST;
			}

		} catch (IOException e) {
			e.printStackTrace();
		} catch (InterruptedException e) {
			e.printStackTrace();
		}
		return TestResult.FAIL_RUN;

	}

	private void dumpOutput(CASTTest _test) throws IOException {

		List<String> buffer = m_outputRunnable.getBuffer();

		// create an output file		
		File output;
		if(!m_noPID)
		    output = new File(NativeProcessLauncher.getProcessID() + "."
				      + _test.getName() + ".cto");
		else
		    output = new File("CTO_" + _test.getName() + ".cto");
		
		if (output.exists()) {
			throw new RuntimeException("Output file already exists: " + output);
		}

		BufferedWriter writer = new BufferedWriter(new FileWriter(output));

		try {
			for (String s : buffer) {
				writer.write(s);
				writer.write('\n');
			}
		} catch (IOException e) {
			e.printStackTrace();
		}

		writer.close();

	}

	private void startOutput(Process p, boolean _showOutput) {
		assert (m_outputThread == null);
		m_outputRunnable = new OutputRunnable(p.getInputStream(), _showOutput);
		m_outputThread = new Thread(m_outputRunnable);
		m_outputThread.start();
	}

	private void stopOutput() {
		assert (m_outputThread != null);
		try {
			// this should be ok... as thread should die naturally
			m_outputThread.join(1000);
			m_outputThread = null;

		} catch (InterruptedException e) {
			e.printStackTrace();
			System.exit(1);
		}

	}

}
