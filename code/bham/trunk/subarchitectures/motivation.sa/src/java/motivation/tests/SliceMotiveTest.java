/**
 * 
 */
package motivation.tests;

import motivation.slice.Motive;
import motivation.slice.MotiveStatus;
import junit.framework.TestCase;

/**
 * @author marc
 *
 */
public class SliceMotiveTest extends TestCase {

	/**
	 * Test method for {@link motivation.slice.Motive#Motive()}.
	 */
	public void testMotive() {
		new Motive();
	}

	/**
	 * Test method for {@link motivation.slice.Motive#Motive(motivation.slice.MotiveStatus, java.lang.String)}.
	 */
	public void testMotiveMotiveStatusString() {
		new Motive(MotiveStatus.UNSURFACED,"test");
	}

}
