package cdsr.rsb;

import java.awt.geom.Line2D;
import java.awt.geom.Point2D;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Iterator;
import java.util.List;
import java.util.concurrent.ExecutionException;
import java.util.logging.Logger;

import rsb.Factory;
import rsb.InitializeException;
import rsb.RSBException;
import rsb.converter.DefaultConverterRepository;
import rsb.converter.ProtocolBufferConverter;
import rsb.patterns.RemoteServer;
import cdsr.marshall.CDSRMarshaller;
import cdsr.objects.ProblemSet;
import cdsr.objects.Room;
import cdsr.objects.SensedObject;
import cdsr.rsb.CdsrMessages.ReceivedRoom;
import cdsr.rsb.CdsrMessages.ReceivedSensedObjects;
import cdsr.rsb.CdsrMessages.SendSensedObjects;


/**
 * Based on an example by swrede.
 * 
 * @author gsh148
 * 
 */
public class SpatialRegionsClient
{

  private static final Logger LOG = Logger.getLogger(SpatialRegionsClient.class
      .getName());

  /**
   * @param args
   * @throws RSBException
   * @throws ExecutionException
   * @throws InterruptedException
   * @throws InitializeException
   */
  public static void main(String[] args) throws InterruptedException,
      ExecutionException, InitializeException
  {
    ProtocolBufferConverter<SendSensedObjects> send_sensed_objects_converter =
        new ProtocolBufferConverter<SendSensedObjects>(SendSensedObjects.getDefaultInstance());
    
    DefaultConverterRepository.getDefaultConverterRepository().addConverter(send_sensed_objects_converter);
    
    ProtocolBufferConverter<ReceivedSensedObjects> received_sensed_objects_converter =
        new ProtocolBufferConverter<ReceivedSensedObjects>(ReceivedSensedObjects.getDefaultInstance());
    
    DefaultConverterRepository.getDefaultConverterRepository().addConverter(received_sensed_objects_converter);
    
    ProtocolBufferConverter<cdsr.rsb.CdsrMessages.Room> room_converter =
        new ProtocolBufferConverter<cdsr.rsb.CdsrMessages.Room>(cdsr.rsb.CdsrMessages.Room.getDefaultInstance());
    
    DefaultConverterRepository.getDefaultConverterRepository().addConverter(room_converter);
    
    ProtocolBufferConverter<ReceivedRoom> received_room_converter =
        new ProtocolBufferConverter<ReceivedRoom>(ReceivedRoom.getDefaultInstance());
    
    DefaultConverterRepository.getDefaultConverterRepository().addConverter(received_room_converter);
    
    // Get remote server object to call exposed request methods of participants
    RemoteServer server = Factory.getInstance().createRemoteServer(
        "/cdsr/server");
    server.activate();
    LOG.info("RemoteServer object activated");

    LOG.info("Calling remote server under scope /cdsr/server:");
    try
    {
//      sendSensedObjects(createTestSensedObjects(), server);
//      sendRoom(createTestRoom(), server);
      ProblemSet ps = loadProblemSet(args[0]);
      sendRoom(ps.getRoom(), server);
      sendSensedObjects(ps.getObjects(), server);
      
    } catch (RSBException e)
    {
      e.printStackTrace();
    }
    server.deactivate();
  }

  private static ProblemSet loadProblemSet(String file_name)
  {
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
  
  private static void sendSensedObjects(ArrayList<SensedObject> sensed_objects, RemoteServer server)
      throws RSBException {
   SendSensedObjects msg = createMessage(sensed_objects);
  
    LOG.info("Data-driven callback (sendSensedObjects) synchronously:");
    
    // Q: is there a time limit on the response? Might be better to use Futures with async calls
    
    ReceivedSensedObjects result = server.call("sendSensedObjects", msg);
    
    System.out.println("response = " + result);
  }

  private static ArrayList<SensedObject> createTestSensedObjects() {
    ArrayList<SensedObject> sensed_objects = new ArrayList<SensedObject>();
    
    ArrayList<Line2D.Double> sides = new ArrayList<Line2D.Double>();
    double[] x = {10, 10, 20, 20};
    double[] y = {10, 20, 20, 10};     
    for (int i=0; i<4; i++)
    {
      sides.add(new Line2D.Double(new Point2D.Double(x[i], y[i]), 
      		new Point2D.Double(x[(i+1)%4], y[(i+1)%4])));    
    }
    sensed_objects.add(new SensedObject("obj1", sides, "desk"));
    return sensed_objects;
  }

  private static void sendRoom(Room room, RemoteServer server) throws RSBException
  {
    cdsr.rsb.CdsrMessages.Room msg = createMessage(room);
    
    LOG.info("Data-driven callback (sendRoom) synchronously:");
    
    // Q: is there a time limit on the response? Might be better to use Futures with async calls
    
    ReceivedRoom result = server.call("sendRoom", msg);
    
    System.out.println("response = " + result);
  }

  private static Room createTestRoom() {
    ArrayList<Line2D.Double> walls = new ArrayList<Line2D.Double>();
    double[] x = {0, 0, 200, 200};
    double[] y = {0, 100, 100, 0};     
    for (int i=0; i<4; i++)
    {
      walls.add(new Line2D.Double(new Point2D.Double(x[i], y[i]), 
          new Point2D.Double(x[(i+1)%4], y[(i+1)%4])));    
    }
    
    Room room = new Room("room1", walls);
    return room;
  }
  
  private static cdsr.rsb.CdsrMessages.Room createMessage(Room room)
  {
    cdsr.rsb.CdsrMessages.Room.Builder room_builder =
        cdsr.rsb.CdsrMessages.Room.newBuilder();
    
    room_builder.setId(room.getID());
    
    for (Iterator<Line2D.Double> iterator = room.iterator(); iterator.hasNext();) {
      Line2D.Double next_wall = (Line2D.Double) iterator.next();
      
      cdsr.rsb.CdsrMessages.Line.Builder line_builder = room_builder.addWallBuilder();
      addLine(next_wall, line_builder);
    }
    
    return room_builder.build();
  }
  
  private static SendSensedObjects createMessage(List<SensedObject> sensed_objects)
  {
    cdsr.rsb.CdsrMessages.SendSensedObjects.Builder send_sensed_objects_builder = 
        SendSensedObjects.newBuilder();
    
    for (Iterator<SensedObject> iterator = sensed_objects.iterator(); iterator.hasNext();)
    {
      SensedObject next_spatial_object = (SensedObject) iterator.next();
      cdsr.rsb.CdsrMessages.SensedObject.Builder spatial_object_builder = 
          send_sensed_objects_builder.addObjectBuilder();
      spatial_object_builder.setId(next_spatial_object.getID());
      spatial_object_builder.setType(next_spatial_object.getType()); // can chain these method calls together
      
      for (Iterator<Line2D.Double> sides_iterator = next_spatial_object.getLines().iterator(); sides_iterator.hasNext();)
      {
        Line2D.Double next_side = (Line2D.Double) sides_iterator.next();
        cdsr.rsb.CdsrMessages.Line.Builder line_builder = spatial_object_builder.addSideBuilder();
        
        addLine(next_side, line_builder);
      }
      
    }
    
    return send_sensed_objects_builder.build();
  }

  private static void addLine(Line2D.Double next_line,
      cdsr.rsb.CdsrMessages.Line.Builder line_builder) {
    cdsr.rsb.CdsrMessages.Point.Builder start_pt_builder = line_builder.getStartBuilder();
    start_pt_builder.setX(next_line.getP1().getX());
    start_pt_builder.setY(next_line.getP1().getY());
    
    cdsr.rsb.CdsrMessages.Point.Builder end_pt_builder = line_builder.getEndBuilder();
    end_pt_builder.setX(next_line.getP2().getX());
    end_pt_builder.setY(next_line.getP2().getY());
  }

}
