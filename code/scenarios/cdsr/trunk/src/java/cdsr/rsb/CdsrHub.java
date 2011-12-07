package cdsr.rsb;

import java.awt.geom.Line2D;
import java.awt.geom.Point2D;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Iterator;
import java.util.List;
import java.util.concurrent.ExecutionException;
import java.util.logging.Logger;

import rsb.AbstractDataHandler;
import rsb.Factory;
import rsb.Informer;
import rsb.InitializeException;
import rsb.Listener;
import rsb.RSBException;
import rsb.converter.DefaultConverterRepository;
import rsb.converter.ProtocolBufferConverter;
import cdsr.marshall.CDSRMarshaller;
import cdsr.objects.ProblemSet;
import cdsr.objects.Room;
import cdsr.objects.SensedObject;
import cdsr.rsb.CdsrMessages.SensedObjects;
import cdsr.rsb.CdsrMessages.SpatialRelation;
import cdsr.rsb.CdsrMessages.SpatialRelations;

public class CdsrHub {
  private static final Logger LOG = Logger.getLogger(CdsrHub.class.getName());

  public static class SpatialRelationsHandler extends
      AbstractDataHandler<SpatialRelations> {
    @Override
    public void handleEvent(SpatialRelations e) {
      try {
        List<SpatialRelation> spatial_relations_list = e.getRelationList();
        System.out.println("SpatialRelations Data received: "
            + spatial_relations_list);

        // TODO write to file / send to Lisp
        // if all 3 components were running then Lisp could listen to the message from C++

      } catch (Exception ex) {
        ex.printStackTrace();
      }
    }

  }

  /**
   * @param args
   * @throws RSBException
   * @throws ExecutionException
   * @throws InterruptedException
   * @throws InitializeException
   */
  public static void main(String[] args) throws InterruptedException,
      ExecutionException, InitializeException {
    String context = null;
    String data_file = null;

    if (args.length == 2) {
      context = args[0];
      data_file = args[1];
    } else {
      System.out.println("You must specify the context and data file");
      System.exit(1);
    }

    registerConverters();

    Informer<cdsr.rsb.CdsrMessages.Room> room_informer = Factory.getInstance()
        .createInformer("/cdsr/room");
    Informer<cdsr.rsb.CdsrMessages.SensedObjects> sensed_objects_informer = Factory
        .getInstance().createInformer("/cdsr/sensedobjects");

    // Activate the informers to be ready for work
    room_informer.activate();
    sensed_objects_informer.activate();

    LOG.info("Informer objects activated");

    try {
      // sendSensedObjects(createTestSensedObjects(), sensed_objects_informer);
      // sendRoom(createTestRoom(), context, room_informer);

      ProblemSet ps = loadProblemSet(data_file);
      sendRoom(ps.getRoom(), context, room_informer);
      sendSensedObjects(ps.getObjects(), sensed_objects_informer);

    } catch (RSBException e) {
      e.printStackTrace();
    }

    // wait for the spatial regions message

    Listener spatial_relations_listener = Factory.getInstance().createListener(
        "/cdsr/spatialrelations");
    spatial_relations_listener.activate();
    spatial_relations_listener.addHandler(new SpatialRelationsHandler(), true);
    
    LOG.info("Listener object activated");
    
    Thread.sleep(10000);

    room_informer.deactivate();
  }

  private static void registerConverters() {
    // Sensed Objects

    ProtocolBufferConverter<SensedObjects> send_sensed_objects_converter = new ProtocolBufferConverter<SensedObjects>(
        SensedObjects.getDefaultInstance());

    DefaultConverterRepository.getDefaultConverterRepository().addConverter(
        send_sensed_objects_converter);

    // Room

    ProtocolBufferConverter<cdsr.rsb.CdsrMessages.Room> room_converter = new ProtocolBufferConverter<cdsr.rsb.CdsrMessages.Room>(
        cdsr.rsb.CdsrMessages.Room.getDefaultInstance());

    DefaultConverterRepository.getDefaultConverterRepository().addConverter(
        room_converter);

    // Spatial Relations

    ProtocolBufferConverter<cdsr.rsb.CdsrMessages.SpatialRelations> spatial_relations_converter = new ProtocolBufferConverter<cdsr.rsb.CdsrMessages.SpatialRelations>(
        cdsr.rsb.CdsrMessages.SpatialRelations.getDefaultInstance());

    DefaultConverterRepository.getDefaultConverterRepository().addConverter(
        spatial_relations_converter);
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

  private static void sendSensedObjects(ArrayList<SensedObject> sensed_objects,
      Informer<SensedObjects> informer) throws RSBException {
    SensedObjects msg = createMessage(sensed_objects);

    LOG.info("sending sensed objects");

    informer.send(msg);
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

  private static void sendRoom(Room room, String context,
      Informer<cdsr.rsb.CdsrMessages.Room> informer) throws RSBException {
    cdsr.rsb.CdsrMessages.Room msg = createMessage(room, context);

    LOG.info("sending room");

    informer.send(msg);

  }

  private static Room createTestRoom() {
    ArrayList<Line2D.Double> walls = new ArrayList<Line2D.Double>();
    double[] x = { 0, 0, 200, 200 };
    double[] y = { 0, 100, 100, 0 };
    for (int i = 0; i < 4; i++) {
      walls.add(new Line2D.Double(new Point2D.Double(x[i], y[i]),
          new Point2D.Double(x[(i + 1) % 4], y[(i + 1) % 4])));
    }

    Room room = new Room("room1", walls);
    return room;
  }

  private static cdsr.rsb.CdsrMessages.Room createMessage(Room room,
      String context) {
    cdsr.rsb.CdsrMessages.Room.Builder room_builder = cdsr.rsb.CdsrMessages.Room
        .newBuilder();

    room_builder.setId(room.getID());
    room_builder.setContext(context);

    for (Iterator<Line2D.Double> iterator = room.iterator(); iterator.hasNext();) {
      Line2D.Double next_wall = (Line2D.Double) iterator.next();

      cdsr.rsb.CdsrMessages.Line.Builder line_builder = room_builder
          .addWallBuilder();
      addLine(next_wall, line_builder);
    }

    return room_builder.build();
  }

  private static SensedObjects createMessage(List<SensedObject> sensed_objects) {
    cdsr.rsb.CdsrMessages.SensedObjects.Builder send_sensed_objects_builder = SensedObjects
        .newBuilder();

    for (Iterator<SensedObject> iterator = sensed_objects.iterator(); iterator
        .hasNext();) {
      SensedObject next_spatial_object = (SensedObject) iterator.next();
      cdsr.rsb.CdsrMessages.SensedObject.Builder spatial_object_builder = send_sensed_objects_builder
          .addObjectBuilder();
      spatial_object_builder.setId(next_spatial_object.getID());
      spatial_object_builder.setType(next_spatial_object.getType()); // can
                                                                     // chain
                                                                     // these
                                                                     // method
                                                                     // calls
                                                                     // together

      for (Iterator<Line2D.Double> sides_iterator = next_spatial_object
          .getLines().iterator(); sides_iterator.hasNext();) {
        Line2D.Double next_side = (Line2D.Double) sides_iterator.next();
        cdsr.rsb.CdsrMessages.Line.Builder line_builder = spatial_object_builder
            .addSideBuilder();

        addLine(next_side, line_builder);
      }

    }

    return send_sensed_objects_builder.build();
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
