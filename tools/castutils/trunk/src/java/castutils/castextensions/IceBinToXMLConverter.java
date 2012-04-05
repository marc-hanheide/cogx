/**
 * @author Nikolaus Demmel <nikolaus@nikolaus-demmel.de>
 * @date 03.04.2012
 */
package castutils.castextensions;

import com.thoughtworks.xstream.XStream;
import com.thoughtworks.xstream.converters.SingleValueConverter;

import java.io.BufferedInputStream;
import java.io.File;
import java.io.FileInputStream;
import java.io.FileNotFoundException;
import java.io.IOException;
import java.io.InputStream;
import java.util.Map;
import java.util.LinkedList;
import java.util.Arrays;


/**
 * @author Nikolaus Demmel <nikolaus@nikolaus-demmel.de>
 */
public final class IceBinToXMLConverter {

  public static void usage() {
    System.out.println("java castutils.castextensions.IceBinToXMLConverter [--full] [inputfile]");
  }  

  public static void main (String[] args) {

    ////////////////////////////////////////////////////////////////////////////
    // Parameters

    boolean verbose = false;
    String filename = "place_properties.bin";

    // Parse parameters
    LinkedList<String> arglist = new LinkedList<String>(Arrays.asList(args));
    verbose = arglist.remove("--full");
      
    // Get filename
    if (arglist.size() > 0) {
      filename = arglist.removeLast();
    }

    // Anything left?
    if (arglist.size() > 0) {
      usage();
      return;
    }



    ////////////////////////////////////////////////////////////////////////////
    // Setup XStream and Ice

    Ice.Communicator ic = Ice.Util.initialize();
    
    XStream xstream = new XStream();

    // By default we display a dense format that is more suitable for human
    // inspection. Pass --full to output in a way that could be read back in.
    if (!verbose) {
      xstream.aliasPackage("", "SpatialProperties");
      xstream.aliasPackage("", "SpatialData");

      xstream.omitField(SpatialProperties.PlaceProperty.class, "distribution");
      xstream.omitField(SpatialProperties.PlaceProperty.class, "inferred");
      xstream.aliasField("place", SpatialProperties.PlaceProperty.class, "placeId");
      xstream.useAttributeFor(SpatialProperties.PlaceProperty.class, "placeId");
      xstream.useAttributeFor(SpatialProperties.PlaceProperty.class, "mapValue");
      xstream.registerConverter(new PropertyValueConverter());
      xstream.useAttributeFor(SpatialProperties.PlaceProperty.class, "mapValueReliable");

      xstream.useAttributeFor(SpatialProperties.ObjectPlaceProperty.class, "category");
      xstream.useAttributeFor(SpatialProperties.ObjectPlaceProperty.class, "supportObjectCategory");
      xstream.aliasField("supportObject", SpatialProperties.ObjectPlaceProperty.class, "supportObjectId");
      xstream.useAttributeFor(SpatialProperties.ObjectPlaceProperty.class, "supportObjectId");
      xstream.useAttributeFor(SpatialProperties.ObjectPlaceProperty.class, "relation");


      xstream.useAttributeFor(SpatialData.ObjectSearchResult.class, "searchedObjectCategory");
      xstream.useAttributeFor(SpatialData.ObjectSearchResult.class, "relation");
      xstream.useAttributeFor(SpatialData.ObjectSearchResult.class, "supportObjectCategory");
      xstream.aliasField("supportObject", SpatialData.ObjectSearchResult.class, "supportObjectId");
      xstream.useAttributeFor(SpatialData.ObjectSearchResult.class, "supportObjectId");
      xstream.aliasField("room", SpatialData.ObjectSearchResult.class, "roomId");
      xstream.useAttributeFor(SpatialData.ObjectSearchResult.class, "roomId");
      xstream.useAttributeFor(SpatialData.ObjectSearchResult.class, "beta");
    }



    ////////////////////////////////////////////////////////////////////////////
    // Let's go

    try {
    
      byte[] data = read(filename);

      Ice.InputStream is = Ice.Util.createInputStream(ic, data);

      try {
        
        Map<String, Ice.Object []> m = castutils.slice.ObjectSeqDictHelper.read(is);
        is.readPendingObjects();
        
        System.out.println(xstream.toXML(m));
        
      } 
      catch (Ice.MarshalException e) {
        System.err.println("Error deserializing Ice objects:");
        System.err.println(e.toString());
        System.exit(1);
      } 
      finally {
        if (is != null)
          is.destroy();
      }

      
    } 
    catch(Exception e) {
      System.err.println("Error:");
      System.err.println(e.toString());
      System.exit(1);
    }
    finally {
      if (ic != null)
        ic.destroy();
    }
  }

  private static byte[] read(String filename) {
    File file = new File(filename);
    byte[] result = new byte[(int)file.length()];
    try {
      InputStream input = null;
      try {
        int total = 0;
        input = new BufferedInputStream(new FileInputStream(filename));
        while(total < result.length){
          int remaining = result.length - total;
          int read = input.read(result, total, remaining); 
          if (read > 0) {
            total += read;
          }
        }
      }
      finally {
        if (input != null)
          input.close();
      }
    }
    catch (FileNotFoundException e) {
      System.err.println("Error: Could not find file '" + filename + "'");
      System.exit(1);
    }
    catch (IOException e) {
      System.err.println("Error while reading file '" + filename + "'");
      System.err.println(e.toString());
      System.exit(1);
    }
    return result;
  }

  private static class PropertyValueConverter implements SingleValueConverter {

    public String toString(Object obj) {
      // hacky code, but it works. We provide one Converter for all property
      // values such that the "mapValue" member of PlaceProperty object can be
      // displayed as an XML attribute
      if (obj.getClass() == SpatialProperties.BinaryValue.class) {
        return Boolean.toString(((SpatialProperties.BinaryValue) obj).value);
      } 
      else if (obj.getClass() == SpatialProperties.IntegerValue.class) {
        return Long.toString(((SpatialProperties.IntegerValue) obj).value);
      }
      else if (obj.getClass() == SpatialProperties.FloatValue.class) {
        return Double.toString(((SpatialProperties.FloatValue) obj).value);
      }
      else if (obj.getClass() == SpatialProperties.StringValue.class) {
        return ((SpatialProperties.StringValue) obj).value;
      } 
      else {
        return "Not Implemented";
      }
      
    }
    
    public Object fromString(String val) {
      // bogus implementation, but we don't care about reading XML back in
      return new SpatialProperties.PropertyValue(); 
    }
    
    public boolean canConvert(Class type) {
      return type.equals(SpatialProperties.PropertyValue.class);
    }
    
  }
}