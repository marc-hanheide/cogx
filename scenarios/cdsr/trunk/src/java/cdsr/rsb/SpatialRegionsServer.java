package cdsr.rsb;

import java.util.List;
import java.util.logging.Logger;

import rsb.Factory;
import rsb.InitializeException;
import rsb.converter.DefaultConverterRepository;
import rsb.converter.ProtocolBufferConverter;
import rsb.patterns.DataCallback;
import rsb.patterns.LocalServer;
import cdsr.rsb.CdsrMessages.ReceivedRoom;
import cdsr.rsb.CdsrMessages.ReceivedSensedObjects;
import cdsr.rsb.CdsrMessages.ReceivedSensedObjects.Builder;
import cdsr.rsb.CdsrMessages.Room;
import cdsr.rsb.CdsrMessages.SendSensedObjects;
import cdsr.rsb.CdsrMessages.SensedObject;

/**
 *
 *
 */
public class SpatialRegionsServer {

  private static final Logger LOG = Logger.getLogger(SpatialRegionsServer.class
      .getName());

  public static class ProcessSensedObjects implements
      DataCallback<ReceivedSensedObjects, SendSensedObjects> {

    @Override
    public ReceivedSensedObjects invoke(SendSensedObjects input)
        throws Throwable {
      List<SensedObject> sp_objs = input.getObjectList();

      System.out.println("received " + sp_objs);

      Builder response = ReceivedSensedObjects.newBuilder();
      response.setNumber(input.getObjectCount());
      return response.build();
    }

  }

  public static class ProcessRoom implements DataCallback<ReceivedRoom, Room> {

    @Override
    public ReceivedRoom invoke(Room input) throws Throwable {
      System.out.println("received room " + input.getId());
      
      cdsr.rsb.CdsrMessages.ReceivedRoom.Builder response =
          ReceivedRoom.newBuilder();
      response.setId(input.getId());
      return response.build();
    }
    
  }
  
  /**
   * @param args
   * @throws InitializeException
   */
  public static void main(String[] args) throws InitializeException {
    ProtocolBufferConverter<SendSensedObjects> send_spatial_objects_converter = new ProtocolBufferConverter<SendSensedObjects>(
        SendSensedObjects.getDefaultInstance());

    DefaultConverterRepository.getDefaultConverterRepository().addConverter(
        send_spatial_objects_converter);

    ProtocolBufferConverter<ReceivedSensedObjects> received_spatial_objects_converter = new ProtocolBufferConverter<ReceivedSensedObjects>(
        ReceivedSensedObjects.getDefaultInstance());

    DefaultConverterRepository.getDefaultConverterRepository().addConverter(
        received_spatial_objects_converter);
    
    ProtocolBufferConverter<Room> room_converter =
        new ProtocolBufferConverter<Room>(Room.getDefaultInstance());
    
    DefaultConverterRepository.getDefaultConverterRepository().addConverter(room_converter);
    
    ProtocolBufferConverter<ReceivedRoom> received_room_converter =
        new ProtocolBufferConverter<ReceivedRoom>(ReceivedRoom.getDefaultInstance());
    
    DefaultConverterRepository.getDefaultConverterRepository().addConverter(received_room_converter);

    // Get local server object which allows to expose request methods to
    // participants
    LocalServer server = Factory.getInstance()
        .createLocalServer("/cdsr/server");
    server.activate();

    // Add methods
    // Callback with handler signature based on event payload
    server.addMethod("sendSensedObjects", new ProcessSensedObjects());
    server.addMethod("sendRoom", new ProcessRoom());

    // Optional: block until server.deactivate or process shutdown
    LOG.info("Server /cdsr/server running");
    server.waitForShutdown();

  }

}
