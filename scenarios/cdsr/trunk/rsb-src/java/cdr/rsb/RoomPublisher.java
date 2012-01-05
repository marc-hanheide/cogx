package cdr.rsb;

import java.util.concurrent.ExecutionException;
import java.util.logging.Logger;

import rsb.Factory;
import rsb.Informer;
import rsb.InitializeException;
import rsb.RSBException;
import cdsr.objects.ProblemSet;
import cdsr.rsb.CdsrMessages;
import cdsr.rsb.CdsrMessages.Room;

public class RoomPublisher {
	private static final Logger LOG = Logger.getLogger(RoomPublisher.class
			.getName());

	/**
	 * @param args
	 * @throws RSBException
	 * @throws ExecutionException
	 * @throws InterruptedException
	 * @throws InitializeException
	 */
	public static void main(String[] args) throws InterruptedException,
			ExecutionException, InitializeException {
		String category = null;
		String data_file = null;

		if (args.length == 2) {
			category = args[0];
			data_file = args[1];
		} else {
			System.out.println("You must specify the category and data file");
			System.exit(1);
		}

		CDSRRSBUtils.registerRoomConverter();

		Informer<cdsr.rsb.CdsrMessages.Room> room_informer = Factory
				.getInstance().createInformer("/cdsr/room");

		// Activate the informers to be ready for work
		room_informer.activate();

		LOG.info("Informer objects activated");

		try {
			ProblemSet ps = CDSRRSBUtils.loadProblemSet(data_file);
			CDSRRSBUtils.sendRoom(ps.getRoom(), category, room_informer);

		} catch (RSBException e) {
			e.printStackTrace();
		}

		room_informer.deactivate();
	}

}
