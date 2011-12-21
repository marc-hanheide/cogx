package cdsr.marshall;

import java.io.File;
import java.io.FileInputStream;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.ObjectInputStream;
import java.io.ObjectOutputStream;
import java.util.ArrayList;

import cdsr.objects.ObjectRelation;
import cdsr.objects.ProblemSet;
import cdsr.objects.Room;
import cdsr.objects.SensedObject;

/**
 * Utility functions for saving a collection of CDSR objects to a file.
 * 
 * @author nah
 * 
 */
public abstract class CDSRMarshaller {

	/**
	 * Convenience function to marshall problem set from just a room and objects
	 * 
	 * @param _filename
	 * @param _room
	 * @param _objects
	 * @throws IOException
	 */
	public static void saveProblemSet(String _filename, Room _room,
			ArrayList<SensedObject> _objects) throws IOException {
		saveProblemSet(_filename, new ProblemSet(_room,
				_objects,
				new ArrayList<ObjectRelation>(0)));
	}

	/**
	 * Convenience function to marshall problem set from just a room
	 * 
	 * @param _filename
	 * @param _room
	 * @param _objects
	 * @throws IOException
	 */
	public static void saveProblemSet(String _filename, Room _room)
			throws IOException {
		saveProblemSet(_filename, new ProblemSet(_room,
				new ArrayList<SensedObject>(0),
				new ArrayList<ObjectRelation>(0)));
	}

	/**
	 * 
	 * @param _filename
	 *            File name to write to. Overwrites if exists.
	 * @param _problem
	 * @throws IOException
	 */
	public static void saveProblemSet(String _filename, ProblemSet _problem)
			throws IOException {

		File file = new File(_filename);

		if (file.exists()) {
			boolean deleted = file.delete();
			if (!deleted) {
				throw new RuntimeException(
						"Could not delete file before writing: " + _filename);
			}
		}

		FileOutputStream fos = new FileOutputStream(file);
		ObjectOutputStream out = new ObjectOutputStream(fos);
		out.writeObject(_problem);
		out.close();
	}

	public static ProblemSet loadProblemSet(String _filename)
			throws IOException, ClassNotFoundException {
		FileInputStream fis = new FileInputStream(_filename);
		ObjectInputStream ois = new ObjectInputStream(fis);
		ProblemSet ps = (ProblemSet) ois.readObject();
		ois.close();
		return ps;
	}

}
