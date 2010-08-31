/** _
 *  part of the ELM system.
 *  @author Dennis Stachowicz
 */

package elm.tests;

import elm.event.EventSpecificBinaryData;

import java.util.Vector;
import java.util.Random;


class BMEventSpecificBinaryData implements EventSpecificBinaryData { // java.io.Serializable {

    byte[] data = null;
    protected BMEventSpecificBinaryData(int size) {

	data = new byte[size];
	Random rng = new Random();
	rng.nextBytes(data);
    }

    public String toString() {
	return "BMEventSpecificBinaryData (length of data field: " + data.length + ")";
    }

}
