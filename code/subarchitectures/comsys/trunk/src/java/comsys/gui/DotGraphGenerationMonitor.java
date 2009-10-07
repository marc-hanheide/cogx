package comsys.gui;

import java.io.BufferedInputStream;
import java.io.*;

/**
 * Monitors whether DOT processes are running on the system
 * @author plison
 *
 */
public class DotGraphGenerationMonitor extends Thread{
	
	PackedLFVisualizerGUI gui;
	
	public DotGraphGenerationMonitor (PackedLFVisualizerGUI gui) {
		this.gui = gui;
	}
	
	synchronized public void run () {
		try {
			while (true) {
			sleep(50);
			String warningText = "(Packed and SDRS graph representations are now being generated, please wait...)";
			if (AreDotProcessesRunning()) {
				gui.setLabelText(warningText);
			}
			else if (gui.getLabelText().equals(warningText)) {
					gui.setLabelText("Graph representations successfully generated");
			}
			
			}
		}
		catch (Exception e) {
			e.printStackTrace();
		}
	}
	
	
	public boolean AreDotProcessesRunning() {
	try {
		String[] cmd = { "/bin/sh", "-c", "ps a | grep dot" };
		Process p = Runtime.getRuntime().exec(cmd);
		sleep(100);
		InputStream is = p.getInputStream(); 
		int occurrences = 0;
		BufferedReader in = new BufferedReader(new InputStreamReader(is));
		while ((in.readLine())!=null){
			occurrences++;
		}
		if (occurrences > 2) {
			return true;
		}
		}
	catch(Exception e){
		//process exception
	}	
		return false;
	}

}
