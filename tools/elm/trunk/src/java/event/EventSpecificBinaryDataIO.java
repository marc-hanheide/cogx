/** _
 *  part of the ELM system.
 *  @author Dennis Stachowicz
 */

package elm.event;

import elm.event.EventSpecificBinaryData;

import java.io.ByteArrayInputStream;
import java.io.ByteArrayOutputStream;
import java.io.IOException;
import java.io.ObjectInputStream;
import java.io.ObjectOutputStream;
import java.io.Serializable;

public class EventSpecificBinaryDataIO {

    public static byte[] toByteArray(EventSpecificBinaryData obj)
	throws IOException {

	return objectToByteArray(obj);
    }


    /**
     *  Serializes an Object to a byte array. 
     *  IMPORTANT: The caller must take care that the object is 
     *             serializable at all!
     */
    public static byte[] objectToByteArray(Object obj)
	throws IOException {

	if (obj == null)
	    return null;

	ByteArrayOutputStream baos = new ByteArrayOutputStream();
	ObjectOutputStream oout = new ObjectOutputStream(baos);
	oout.writeObject(obj);
	oout.close();
	return baos.toByteArray();
    }

    

    public static EventSpecificBinaryData fromByteArray(byte[] byteArray)
	throws IOException, ClassNotFoundException, ClassCastException {

	return (EventSpecificBinaryData) objectFromByteArray(byteArray);
    }


    public static Object objectFromByteArray(byte[] byteArray)
	throws IOException, ClassNotFoundException, ClassCastException {

	if (byteArray != null) {

	    ObjectInputStream objectIn = 
		new ObjectInputStream(new ByteArrayInputStream(byteArray));
	    return objectIn.readObject();
	}
	return null;
    }

}
