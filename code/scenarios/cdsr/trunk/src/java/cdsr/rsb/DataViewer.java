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
import cdsr.rsb.CdsrMessages.Room;

public class DataViewer extends StandaloneViewer {

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

		ProblemSet ps = CDSRMarshaller.loadProblemSet(args[0]);

		DataViewer viewer = new DataViewer();
		viewer.addProblemSet(ps);

		try {

			CDSRRSBUtils.registerRoomConverter();
			
			Room room = CDSRRSBUtils.createRoomMessage(ps.getRoom(),
					"Classroom");

			RemoteServer server = Factory.getInstance().createRemoteServer(
					"/cdsr/rooms");
			server.activate();

			LOG.info("making call");
			server.call("standardiseCoordinateFrame", room);
			LOG.info("done");
			
			server.deactivate();
		} catch (RSBException e) {
			e.printStackTrace();
		}

	}

}
