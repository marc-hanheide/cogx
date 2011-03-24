/**
 * 
 */
package castutils.experimentation;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileReader;
import java.io.IOException;
import java.util.Map;
import java.util.StringTokenizer;

import org.apache.log4j.Logger;

import cast.architecture.ManagedComponent;

/**
 * @author marc
 * 
 */
public class FileLogger extends ManagedComponent implements FileListener {

	private String loggerPrefix;
	private FileMonitor watchedFile;

	public FileLogger() {
		watchedFile = new FileMonitor(500);
	}

	@Override
	public void fileChanged(File file) {
		try {
			
			BufferedReader fr = new BufferedReader(new FileReader(file));

			// Here BufferedInputStream is added for fast reading.
			StringBuffer content = new StringBuffer();

			while (fr.ready()) {
				content.append(fr.readLine() + "\n");
			}
			Logger.getLogger(loggerPrefix+"."+file.getName()).info(content.toString());

		} catch (FileNotFoundException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		} catch (IOException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}

	}

	/*
	 * (non-Javadoc)
	 * 
	 * @see cast.core.CASTComponent#stop()
	 */
	@Override
	protected void stop() {
		watchedFile.stop();
	}

	/*
	 * (non-Javadoc)
	 * 
	 * @see cast.core.CASTComponent#configure(java.util.Map)
	 */
	@Override
	protected void configure(Map<String, String> config) {
		String loggerName = config.get("-logger");
		if (loggerName!=null)
			loggerPrefix = loggerName;
		else
			loggerPrefix = FileLogger.class.getSimpleName();
		String fileNames = config.get("-files");
		if (fileNames != null) {
			StringTokenizer st = new StringTokenizer(fileNames, " ");
			while (st.hasMoreTokens()) {
				String fn = st.nextToken();
				Logger.getLogger(loggerPrefix).info("watching file "+fn);
				watchedFile.addFile(new File(fn));
			}
			watchedFile.addListener(this);
		}
	}

}
