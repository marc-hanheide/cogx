package castutils.castextensions;

import java.util.HashMap;
import java.util.Map;

import org.apache.log4j.PatternLayout;
import org.apache.log4j.spi.LoggingEvent;

/**
 * a colored PatternLayout that uses pre-defined color maps. It assign a color
 * to each logger from the set of defined colors. If all colors of the set are
 * in use, the assignemtn of colors starts from the beginning.
 * 
 * @author marc
 * 
 */
public class ColorPatternLayout extends PatternLayout {

	/**
	 * the colorset in ANSI termianl escaped commands
	 * 
	 */
	private final static String[] COLORSET = { "\u001B[0;31m\u001B[0;47m", // red/white
			"\u001B[0;32m\u001B[0;47m", // green/white
			"\u001B[0;33m\u001B[0;40m", // yellow/black
			"\u001B[0;34m\u001B[0;47m", // blue/white
			"\u001B[0;35m\u001B[0;47m", // magenta/white
			"\u001B[0;36m\u001B[0;47m", // cyan/white
			"\u001B[0;31m\u001B[0;40m", // red/black
			"\u001B[0;32m\u001B[0;40m", // green/black
			"\u001B[0;34m\u001B[0;40m", // blue/black
			"\u001B[0;35m\u001B[0;40m", // magenta/black
			"\u001B[0;36m\u001B[0;40m",	// cyan/black
			"\u001B[0;31m\u001B[0;43m", // red/yellow
			"\u001B[0;32m\u001B[0;43m", // green/yellow
			"\u001B[0;34m\u001B[0;43m", // blue/yellow
			"\u001B[0;35m\u001B[0;43m", // magenta/yellow
			"\u001B[0;36m\u001B[0;43m" 	// cyan/yellow
	};
	/**
	 * the default color
	 * 
	 */
	private final static String DEFAULTCOLOR = "\u001B[0;30m\u001B[0;47m";

	/**
	 * map of color assignments to logger names
	 * 
	 */
	private Map<String, String> componentColorMap;

	/**
	 * cycle through all assign colors
	 * 
	 */
	private int colorCycle = 0;

	public ColorPatternLayout() {
		componentColorMap = new HashMap<String, String>();
		colorCycle = 0;
	}

	public ColorPatternLayout(String string) {
		super(string);
		componentColorMap = new HashMap<String, String>();
		colorCycle = 0;
	}

	/**
	 * actually do the formatting by first appending the appropriate color, then
	 * pass to PatternLayout and reset to default color afterwards.
	 * 
	 */
	@Override
	public String format(LoggingEvent loggingEvent) {
		String color = componentColorMap.get(loggingEvent.getLoggerName());
		if (color == null) {
			componentColorMap.put(loggingEvent.getLoggerName(),
					COLORSET[colorCycle]);
			color = COLORSET[colorCycle];
			// increase cycle and make sure we start from the beginning, if we
			// reached the end
			colorCycle++;
			if (colorCycle >= COLORSET.length)
				colorCycle = 0;
		}
		StringBuffer oBuffer = new StringBuffer();
		oBuffer.append(color);
		oBuffer.append(super.format(loggingEvent));
		oBuffer.append(DEFAULTCOLOR);
		return oBuffer.toString();
	}

}