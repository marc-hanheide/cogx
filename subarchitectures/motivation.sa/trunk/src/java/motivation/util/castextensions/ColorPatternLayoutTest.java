/**
 * 
 */
package motivation.util.castextensions;

import static org.junit.Assert.*;

import java.util.logging.Logger;

import org.apache.commons.logging.impl.Log4JLogger;
import org.apache.log4j.Appender;
import org.apache.log4j.Category;
import org.apache.log4j.ConsoleAppender;
import org.apache.log4j.Level;
import org.apache.log4j.spi.LoggingEvent;
import org.junit.Test;

/**
 * @author marc
 *
 */
public class ColorPatternLayoutTest {

	private ColorPatternLayoutTest layout;

	/**
	 * Test method for {@link motivation.util.castextensions.ColorPatternLayout#ColorPatternLayout()}.
	 */
	@Test
	public void testColorPatternLayout() {
		layout=new ColorPatternLayoutTest();
	}

	/**
	 * Test method for {@link motivation.util.castextensions.ColorPatternLayout#format(org.apache.log4j.spi.LoggingEvent)}.
	 */
	@Test
	public void testFormatLoggingEvent() {
		Category cat1 = Category.getInstance("cat 1");
		Category cat2 = Category.getInstance("cat 2");
		ConsoleAppender app1 = new ConsoleAppender(new ColorPatternLayout());
		app1.setName("test appender 1");
		app1.setTarget(ConsoleAppender.SYSTEM_OUT);
		cat1.addAppender(app1);
		cat2.addAppender(app1);
		
		
		cat1.error("hurga 1");
		cat2.error("hurga 2");

		System.out.println("yes");
		
	}
	
	

}
