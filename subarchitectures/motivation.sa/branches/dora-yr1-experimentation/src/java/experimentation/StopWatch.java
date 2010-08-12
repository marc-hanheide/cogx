package experimentation;

import org.apache.log4j.Logger;
import motivation.util.castextensions.XMLTag;

public class StopWatch {

    private Logger loggerInstance;
    private Logger infoLoggerInstance;
    long lastTic;
    int count;
//	double cumulatedTime;
//	double cumSquaredTime;
    long lastSpan;
    boolean running;

    public StopWatch(String name) {
        loggerInstance = Logger.getLogger("StopWatch.tictoc." + name);
        infoLoggerInstance = Logger.getLogger("StopWatch.info." + name);
        count = 0;
//		cumulatedTime = 0;
    }

    public synchronized void tic() {
        lastTic = System.currentTimeMillis();
        running = true;
    }

    public void toc() {
        toc("");
    }

    public synchronized boolean isRunning() {
        return running;
    }

    public void info(String text) {
        infoLoggerInstance.info("<STOPWATCHINFO>"+XMLTag.escapeCdata(text)+"</STOPWATCHINFO>");
    }

    public synchronized void toc(String text) {
        if (!running) {
            throw (new IllegalStateException("StopWatch not running!"));
        }
        count++;
        lastSpan = System.currentTimeMillis() - lastTic;
//		cumulatedTime += lastSpan;
//		cumSquaredTime += lastSpan * lastSpan;

        XMLTag t = new XMLTag("STOPWATCH");
        if (!"".equals(text)) {
            t.addAttr("message", XMLTag.escapeString(text));
        }
        t.addContents(Long.toString(lastSpan), false);



//		loggerInstance.info(text + ": " + (Long.toString(lastSpan)));
        loggerInstance.info(t.toString());
        running = false;
    }
//	public void stats() {
//		loggerInstance.info("cum. time elapsed: "
//				+ (Double.toString(cumulatedTime)));
//		loggerInstance.info("count:             " + (Integer.toString(count)));
//		double mean = cumulatedTime / count;
//		double var = cumSquaredTime / count - (mean * mean);
//		loggerInstance.info("mean time elapsed: " + (Double.toString(mean)));
//		loggerInstance.info("var. time elapsed: " + (Double.toString(var)));
//	}
}
