package cdsr.rsb;

import java.io.IOException;
import java.util.logging.Logger;

import rsb.Factory;
import rsb.InitializeException;
import rsb.RSBException;
import rsb.patterns.RemoteServer;
import cdsr.gui.StandaloneViewer;
import cdsr.marshall.CDSRMarshaller;
import cdsr.objects.ProblemSet;
import cdsr.rsb.CdsrMessages.RoomWithObjects;

public class DataViewer extends StandaloneViewer {

	/**
	 * 
	 */
	private static final long serialVersionUID = 1L;
	
	private static final Logger LOG = Logger.getLogger(DataViewer.class
			.getName());

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

		ProblemSet originalPS = CDSRMarshaller.loadProblemSet(args[0]);

		DataViewer viewer = new DataViewer();
//		viewer.addProblemSet(originalPS);

		try {

			CDSRRSBUtils.registerRoomWithObjectsConverter();
			
			RoomWithObjects room = CDSRRSBUtils.createRoomWithObjectsMessage(originalPS,
					"Classroom");

			RemoteServer server = Factory.getInstance().createRemoteServer(
					"/cdsr/rooms");
			server.activate();

			LOG.info("making call");
			RoomWithObjects transformedPS = server.call("standardiseCoordinateFrame", room);
			viewer.addProblemSet(CDSRRSBUtils.toProblemSet(transformedPS));
			
			LOG.info("done");
			
			server.deactivate();
		} catch (RSBException e) {
			e.printStackTrace();
		}

	}

}
