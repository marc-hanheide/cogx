package cdsr.marshall;

import java.awt.geom.Line2D;
import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.text.NumberFormat;
import java.util.ArrayList;
import java.util.Iterator;

import cdsr.objects.ObjectRelation;
import cdsr.objects.ProblemSet;
import cdsr.objects.Room;
import cdsr.objects.SensedObject;

/**
 * Converts a serialized ProblemSet into space separated files for use by Lisp.
 * 
 * @author ghorn
 * 
 */
public class ProblemSetConverter {

	private static NumberFormat LINE_POINT_FORMAT = NumberFormat.getInstance();

	static {
		LINE_POINT_FORMAT.setMaximumFractionDigits(3);
	}

	/**
	 * @param args
	 * @throws ClassNotFoundException
	 * @throws IOException
	 */
	public static void main(String[] args) throws IOException,
			ClassNotFoundException {
		if (args.length == 2) {
			ProblemSet problem_set = CDSRMarshaller.loadProblemSet(args[0]);

			// // create the output filenames
			// String stem = args[0].substring(0, args[0].length()-4);
			// System.out.println(stem);

			saveSensedObjects(args[0] + "_obj", problem_set.getObjects());
			saveRoom(args[0] + "_room", problem_set.getRoom(), args[1]);
			saveRelations(args[0] + "_rlns", problem_set.getRelations());
		} else {
			System.out
					.println("Expect a 2 arguments: the filename of the problem set and the room category");
		}

	}

	public static void saveRelations(String _filename, ArrayList<ObjectRelation> object_relations) throws IOException
	{
		File file = new File(_filename);

		if (file.exists()) {
			boolean deleted = file.delete();
			if (!deleted) {
				throw new RuntimeException(
						"Could not delete file before writing: " + _filename);
			}
		}

		BufferedWriter bw = new BufferedWriter(new FileWriter(file));
		bw.write(object_relations.size() + "");
		bw.newLine();

		for (ObjectRelation object_relation : object_relations) {
			
			bw.write(formatID(object_relation.getID()) + " " + object_relation.getType());
			bw.write(" " + formatID(object_relation.getStart()) +
				" " + formatID(object_relation.getEnd()) + " " +
					object_relation.getStrength()); 
			bw.newLine();
		}

		bw.flush();
		bw.close();
	}
	

	public static String formatID(String input_id)
	{
		return input_id.replace(':', '-');
	}

	public static void saveRoom(String _filename, Room _room,
			String room_category) throws IOException {
		File file = new File(_filename);

		if (file.exists()) {
			boolean deleted = file.delete();
			if (!deleted) {
				throw new RuntimeException(
						"Could not delete file before writing: " + _filename);
			}
		}

		BufferedWriter bw = new BufferedWriter(new FileWriter(file));
		bw.write(formatID(_room.getID()) + " " + room_category);
		bw.newLine();
		bw.write(_room.getLines().size() + "");
		bw.newLine();

		Iterator<Line2D.Double> line_itr = _room.iterator();
		while (line_itr.hasNext()) {
			Line2D.Double next_line = line_itr.next();
			bw.write(" " + LINE_POINT_FORMAT.format(next_line.getX1()) + " "
					+ LINE_POINT_FORMAT.format(next_line.getY1()));
			bw.write(" " + LINE_POINT_FORMAT.format(next_line.getX2()) + " "
					+ LINE_POINT_FORMAT.format(next_line.getY2()));
			bw.newLine();
		}

		bw.flush();
		bw.close();
	}

	public static void saveSensedObjects(String _filename,
			ArrayList<SensedObject> sensed_objects) throws IOException {
		File file = new File(_filename);

		if (file.exists()) {
			boolean deleted = file.delete();
			if (!deleted) {
				throw new RuntimeException(
						"Could not delete file before writing: " + _filename);
			}
		}

		BufferedWriter bw = new BufferedWriter(new FileWriter(file));
		bw.write(sensed_objects.size() + "");
		bw.newLine();

		for (SensedObject sensed_object : sensed_objects) {
			// re-format ID - replace colon with hyphen
			bw.write(formatID(sensed_object.getID()));
			bw.write(" ");
			// reformat type - remove trailing digits
			bw.write(formatObjectType(sensed_object.getType()));
			Iterator<Line2D.Double> line_itr = sensed_object.iterator();
			while (line_itr.hasNext()) {
				Line2D.Double next_line = line_itr.next();
				bw.write(" " + LINE_POINT_FORMAT.format(next_line.getX1())
						+ " " + LINE_POINT_FORMAT.format(next_line.getY1()));
				bw.write(" " + LINE_POINT_FORMAT.format(next_line.getX2())
						+ " " + LINE_POINT_FORMAT.format(next_line.getY2()));
			}
			bw.newLine();
		}

		bw.flush();
		bw.close();
	}

	public static String formatObjectType(String input_type)
	{
		StringBuilder builder = new StringBuilder();
    		for (int i = 0; i < input_type.length(); i++) {
        		char c = input_type.charAt(i);
        		if (!Character.isDigit(c)) {
            			builder.append(c);
        		}
    		}
    		return builder.toString();
	}
}
