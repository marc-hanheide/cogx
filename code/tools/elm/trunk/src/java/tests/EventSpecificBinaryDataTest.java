/** _
 *  part of the ELM system.
 *  @author Dennis Stachowicz
 */

package elm.tests;

import elm.event.EventSpecificBinaryData;

import java.util.Vector;
import java.util.Random;


class EventSpecificBinaryDataTest implements EventSpecificBinaryData { // java.io.Serializable {

    String testString = "This is a test string.";
    /* 
    Vector<Integer> data = new Vector<Integer>();

    protected EventSpecificBinaryDataTest(int size) {
	Random rng = new Random();

	for (int i = 0; i < size; i++)
	    data.add(new Integer(rng.nextInt()));
    }
    */
    byte[] data = null;
    protected EventSpecificBinaryDataTest(int size) {

	data = new byte[size];
	Random rng = new Random();
	rng.nextBytes(data);
    }

    public String toString() {
	return "EventSpecificBinaryDataTest: (testString = \"" + testString + "\", data = " + data + ")";
    }

}
