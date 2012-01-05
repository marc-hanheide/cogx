package cdr.rsb;

import java.awt.geom.Line2D;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Iterator;
import java.util.List;
import java.util.logging.Logger;

import rsb.Informer;
import rsb.RSBException;
import rsb.converter.DefaultConverterRepository;
import rsb.converter.ProtocolBufferConverter;
import cdsr.marshall.CDSRMarshaller;
import cdsr.objects.ObjectRelation;
import cdsr.objects.ProblemSet;
import cdsr.objects.Room;
import cdsr.objects.SensedObject;
import cdsr.rsb.CdsrMessages;
import cdsr.rsb.CdsrMessages.Line;
import cdsr.rsb.CdsrMessages.Point;
import cdsr.rsb.CdsrMessages.RoomWithObjects;
import cdsr.rsb.CdsrMessages.SensedObject.Builder;
import cdsr.rsb.CdsrMessages.SpatialRelation;

public abstract class CDSRRSBUtils {
	private static final Logger LOG = Logger.getLogger(CDSRRSBUtils.class
			.getName());

	public static void registerRoomConverter() {

		ProtocolBufferConverter<cdsr.rsb.CdsrMessages.Room> room_converter = new ProtocolBufferConverter<cdsr.rsb.CdsrMessages.Room>(
				cdsr.rsb.CdsrMessages.Room.getDefaultInstance());

		DefaultConverterRepository.getDefaultConverterRepository()
				.addConverter(room_converter);

	}

	public static void registerRoomWithObjectsConverter() {

		ProtocolBufferConverter<cdsr.rsb.CdsrMessages.RoomWithObjects> room_converter = new ProtocolBufferConverter<cdsr.rsb.CdsrMessages.RoomWithObjects>(
				cdsr.rsb.CdsrMessages.RoomWithObjects.getDefaultInstance());

		DefaultConverterRepository.getDefaultConverterRepository()
				.addConverter(room_converter);

	}

	public static RoomWithObjects createRoomWithObjectsMessage(ProblemSet _ps,
			String _roomCategory) {
		cdsr.rsb.CdsrMessages.RoomWithObjects.Builder builder = RoomWithObjects
				.newBuilder();
		builder.setRoom(createRoomMessage(_ps.getRoom(), _roomCategory));

		for (SensedObject object : _ps.getObjects()) {
			builder.addObject(createObjectMessage(object));
		}

		return builder.build();
	}

	public static cdsr.rsb.CdsrMessages.SensedObject createObjectMessage(
			SensedObject _object) {
		Builder builder = cdsr.rsb.CdsrMessages.SensedObject.newBuilder();

		builder.setId(_object.getID());
		builder.setType(_object.getType());

		for (Line2D.Double side : _object) {
			cdsr.rsb.CdsrMessages.Line.Builder line_builder = builder
					.addSideBuilder();
			addLine(side, line_builder);
		}

		return builder.build();
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

	public static ProblemSet toProblemSet(RoomWithObjects _rwo) {
		return new ProblemSet(toRoom(_rwo.getRoom()),
				toSensedObjectList(_rwo.getObjectList()),
				toObjectRelationList(_rwo.getRelationList()));
	}

	public static ArrayList<ObjectRelation> toObjectRelationList(
			List<SpatialRelation> _relationList) {
		// TODO add relation stuff
		return null;
	}

	public static ArrayList<SensedObject> toSensedObjectList(
			List<cdsr.rsb.CdsrMessages.SensedObject> _objectList) {
		ArrayList<SensedObject> objects = new ArrayList<SensedObject>(
				_objectList.size());
		for (cdsr.rsb.CdsrMessages.SensedObject object : _objectList) {
			objects.add(new SensedObject(object.getId(), toLineList(object
					.getSideList()), object.getType()));
		}
		return objects;
	}

	public static Room toRoom(cdsr.rsb.CdsrMessages.Room _room) {
		return new Room(_room.getId(), toLineList(_room.getWallList()));
	}

	public static ArrayList<Line2D.Double> toLineList(List<Line> _lineList) {
		ArrayList<Line2D.Double> lines = new ArrayList<Line2D.Double>(
				_lineList.size());

		for (Line line : _lineList) {
			lines.add(new Line2D.Double(line.getStart().getX(), line.getStart()
					.getY(), line.getEnd().getX(), line.getEnd().getY()));
		}

		return lines;
	}

}
