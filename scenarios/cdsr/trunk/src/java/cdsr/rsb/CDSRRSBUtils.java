package cdsr.rsb;

import java.awt.geom.Line2D;
import java.io.IOException;
import java.util.Iterator;
import java.util.logging.Logger;

import rsb.Informer;
import rsb.RSBException;
import rsb.converter.DefaultConverterRepository;
import rsb.converter.ProtocolBufferConverter;
import cdsr.marshall.CDSRMarshaller;
import cdsr.objects.ProblemSet;
import cdsr.objects.Room;

public abstract class CDSRRSBUtils {
	private static final Logger LOG = Logger.getLogger(CDSRRSBUtils.class
			.getName());


	public static void registerRoomConverter() {

		ProtocolBufferConverter<cdsr.rsb.CdsrMessages.Room> room_converter = new ProtocolBufferConverter<cdsr.rsb.CdsrMessages.Room>(
				cdsr.rsb.CdsrMessages.Room.getDefaultInstance());

		DefaultConverterRepository.getDefaultConverterRepository()
				.addConverter(room_converter);

	}

	public static ProblemSet loadProblemSet(String _filename) {
		try {
			return CDSRMarshaller.loadProblemSet(_filename);
		} catch (IOException e) {
			e.printStackTrace();
		} catch (ClassNotFoundException e) {
			e.printStackTrace();
		}
		return null;
	}

	public static void sendRoom(Room _room, String _category,
			Informer<cdsr.rsb.CdsrMessages.Room> _informer) throws RSBException {
		cdsr.rsb.CdsrMessages.Room msg = createRoomMessage(_room, _category);

		LOG.info("sending room");

		_informer.send(msg);
	}

	public static cdsr.rsb.CdsrMessages.Room createRoomMessage(Room room,
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

	public static void addLine(Line2D.Double next_line,
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
