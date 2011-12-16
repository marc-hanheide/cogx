package cdsr.rsb;

import java.awt.geom.Line2D;
import java.io.IOException;
import java.util.Iterator;
import java.util.List;
import java.util.concurrent.ExecutionException;
import java.util.logging.Logger;

import rsb.Factory;
import rsb.Informer;
import rsb.InitializeException;
import rsb.RSBException;
import rsb.converter.DefaultConverterRepository;
import rsb.converter.ProtocolBufferConverter;
import cdsr.marshall.CDSRMarshaller;
import cdsr.objects.ProblemSet;
import cdsr.objects.Room;
import cdsr.objects.SensedObject;
import cdsr.rsb.CdsrMessages.SensedObjects;

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

		registerConverters();

		Informer<cdsr.rsb.CdsrMessages.Room> room_informer = Factory
				.getInstance().createInformer("/cdsr/room");

		// Activate the informers to be ready for work
		room_informer.activate();

		LOG.info("Informer objects activated");

		try {
			ProblemSet ps = loadProblemSet(data_file);
			sendRoom(ps.getRoom(), category, room_informer);

		} catch (RSBException e) {
			e.printStackTrace();
		}


		room_informer.deactivate();
	}

	private static void registerConverters() {

		// Room

		ProtocolBufferConverter<cdsr.rsb.CdsrMessages.Room> room_converter = new ProtocolBufferConverter<cdsr.rsb.CdsrMessages.Room>(
				cdsr.rsb.CdsrMessages.Room.getDefaultInstance());

		DefaultConverterRepository.getDefaultConverterRepository()
				.addConverter(room_converter);

	}

	private static ProblemSet loadProblemSet(String file_name) {
		try {
			return CDSRMarshaller.loadProblemSet(file_name);
		} catch (IOException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		} catch (ClassNotFoundException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
		return null;
	}

	private static void sendRoom(Room room, String category,
			Informer<cdsr.rsb.CdsrMessages.Room> informer) throws RSBException {
		cdsr.rsb.CdsrMessages.Room msg = createMessage(room, category);

		LOG.info("sending room");

		informer.send(msg);

	}

	private static cdsr.rsb.CdsrMessages.Room createMessage(Room room,
			String category) {
		cdsr.rsb.CdsrMessages.Room.Builder room_builder = cdsr.rsb.CdsrMessages.Room
				.newBuilder();

		room_builder.setId(room.getID());
		room_builder.setCategory(category);

		for (Iterator<Line2D.Double> iterator = room.iterator(); iterator
				.hasNext();) {
			Line2D.Double next_wall = (Line2D.Double) iterator.next();

			cdsr.rsb.CdsrMessages.Line.Builder line_builder = room_builder
					.addWallBuilder();
			addLine(next_wall, line_builder);
		}

		return room_builder.build();
	}

	private static void addLine(Line2D.Double next_line,
			cdsr.rsb.CdsrMessages.Line.Builder line_builder) {
		cdsr.rsb.CdsrMessages.Point.Builder start_pt_builder = line_builder
				.getStartBuilder();
		start_pt_builder.setX(next_line.getP1().getX());
		start_pt_builder.setY(next_line.getP1().getY());

		cdsr.rsb.CdsrMessages.Point.Builder end_pt_builder = line_builder
				.getEndBuilder();
		end_pt_builder.setX(next_line.getP2().getX());
		end_pt_builder.setY(next_line.getP2().getY());
	}

}
