package cdsr.rsb;

import java.awt.geom.Line2D;
import java.awt.geom.Point2D;
import java.io.IOException;
import java.util.ArrayList;
import java.util.logging.Logger;

import rsb.Factory;
import rsb.InitializeException;
import rsb.RSBException;
import rsb.patterns.RemoteServer;
import cdsr.gui.StandaloneViewer;
import cdsr.marshall.CDSRMarshaller;
import cdsr.objects.ProblemSet;
import cdsr.objects.Room;
import cdsr.objects.SensedObject;
import cdsr.rsb.CdsrMessages;
import cdsr.rsb.CdsrMessages.RoomWithObjects;

public class DataViewer extends StandaloneViewer {

	/**
	 * 
	 */
	private static final long serialVersionUID = 1L;

	private static final Logger LOG = Logger.getLogger(DataViewer.class
			.getName());

	private static Room createTestRoom() {
		ArrayList<Line2D.Double> walls = new ArrayList<Line2D.Double>();
		double[] x = { 0, 0, 5, 5 };
		double[] y = { 0, 2, 4, 2 };
		for (int i = 0; i < 4; i++) {
			walls.add(new Line2D.Double(new Point2D.Double(x[i], y[i]),
					new Point2D.Double(x[(i + 1) % 4], y[(i + 1) % 4])));
		}

		Room room = new Room("room1", walls);
		return room;
	}

	private static ArrayList<SensedObject> createTestSensedObjects() {
		ArrayList<SensedObject> sensed_objects = new ArrayList<SensedObject>();

		ArrayList<Line2D.Double> sides = new ArrayList<Line2D.Double>();
		double[] x = { 10, 10, 20, 20 };
		double[] y = { 10, 20, 20, 10 };
		for (int i = 0; i < 4; i++) {
			sides.add(new Line2D.Double(new Point2D.Double(x[i], y[i]),
					new Point2D.Double(x[(i + 1) % 4], y[(i + 1) % 4])));
		}
		sensed_objects.add(new SensedObject("obj1", sides, "desk"));
		return sensed_objects;
	}
	
	private static ProblemSet createTestProblemSet() {
		return new ProblemSet(createTestRoom(), createTestSensedObjects(), null);
	}

	/**
	 * @param args
	 * @throws ClassNotFoundException
	 * @throws IOException
	 * @throws InitializeException
	 */
	public static void main(String[] args) throws IOException,
			ClassNotFoundException {

		if (args.length != 1) {
			System.out
					.println("Only one arguement allowed/required, path to .cdsr save file.");
			return;
		}

//		ProblemSet originalPS = CDSRMarshaller.loadProblemSet(args[0]);
		ProblemSet originalPS = createTestProblemSet();
		
		DataViewer viewer = new DataViewer();
		// viewer.addProblemSet(originalPS);

		try {

			CDSRRSBUtils.registerRoomWithObjectsConverter();

			RoomWithObjects room = CDSRRSBUtils.createRoomWithObjectsMessage(
					originalPS, "Classroom");

			RemoteServer server = Factory.getInstance().createRemoteServer(
					"/cdsr/rooms");
			server.activate();

			LOG.info("making call");
			RoomWithObjects transformedPS = server.call(
					"standardiseCoordinateFrame", room);
			viewer.addProblemSet(CDSRRSBUtils.toProblemSet(transformedPS));

			LOG.info("done");

			server.deactivate();
		} catch (RSBException e) {
			e.printStackTrace();
		}

	}

}
