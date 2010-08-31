/** _
 *  part of the ELM system.
 *  @author Dennis Stachowicz
 */

package elm.event;

import java.util.Collection;
import java.util.Iterator;

import com.vividsolutions.jts.geom.PrecisionModel;
import com.vividsolutions.jts.geom.GeometryFactory;
import com.vividsolutions.jts.geom.Geometry;
import com.vividsolutions.jts.geom.GeometryCollection;
import com.vividsolutions.jts.geom.Point;
import com.vividsolutions.jts.geom.LineString;
import com.vividsolutions.jts.geom.impl.PackedCoordinateSequence;
import com.vividsolutions.jts.io.WKTReader;


public class EventLocationFactory {

    public static final int defaultSRID	          = -1;
    public static final int defaultBufferSegments = -1;
    
    // the factory might need adjustment later on (SRID, precision model, etc.)
    private GeometryFactory gFactory 	          = null;
    private WKTReader wktReader		          = null;
    
    // uses the default SRID
    public EventLocationFactory() {
	init(defaultSRID);
    }


    public EventLocationFactory(int srid) {
	init(srid);
    }

    private void init(int srid) {
	gFactory  = new GeometryFactory(new PrecisionModel(), srid);
	wktReader = new WKTReader(gFactory);
    }

    
    public int getSRID() {
	return gFactory.getSRID();
    }

    // uses a default value for bufferSegments
    public EventLocation fromPoint(double[] coordinates, double ptBufferDistance) {
	return fromPoint(coordinates, ptBufferDistance, defaultBufferSegments);
    }
    
    public EventLocation fromPoint(double[] coordinates, 
				   double ptBufferDistance, 
				   int bufferSegments) {
	
	PackedCoordinateSequence.Double dsequence = 
	    new PackedCoordinateSequence.Double.Double(coordinates, 
						       coordinates.length);
	Point point = new Point(dsequence, gFactory);
	Geometry geometry = null;
	if (bufferSegments == -1)
	    geometry = point.buffer(ptBufferDistance);
	else
	    geometry = point.buffer(ptBufferDistance, bufferSegments);
	return new EventLocation(geometry, this);			
    }


    // uses a default value for bufferSegments
    public EventLocation fromLinestring(double[][] coordinatesArray, 
					double bufferDistance)
	throws ArrayIndexOutOfBoundsException {

	return fromLinestring(coordinatesArray, bufferDistance, defaultBufferSegments);
    }
    
    public EventLocation fromLinestring(double[][] coordinatesArray, 
					double bufferDistance, 
					int bufferSegments) 
	throws ArrayIndexOutOfBoundsException {

	// JTS does not seem to handle linestrings of length 1 gracefully
	// so we use a point then:
	if (coordinatesArray.length == 1)
	    return fromPoint(coordinatesArray[0], bufferDistance, defaultBufferSegments);

	// now the standard case...
	int dim = coordinatesArray[0].length;
	double[] flatCoordinates = new double[coordinatesArray.length * dim];
	for (int i = 0; i < coordinatesArray.length; i++) {
	    if (coordinatesArray[i].length != dim)
		throw new ArrayIndexOutOfBoundsException("coordinatesArray must " + 
							 "only include sub-arrays " + 
							 "of the same dimension!");
	    for (int j = 0; j < coordinatesArray[i].length; j++)
		flatCoordinates[i * dim + j] = coordinatesArray[i][j];
	}	
	
	PackedCoordinateSequence.Double dsequence = 
	    new PackedCoordinateSequence.Double.Double(flatCoordinates, dim);

	LineString ls = new LineString(dsequence, gFactory);
	Geometry geometry = null;
	if (bufferSegments == -1)
	    geometry = ls.buffer(bufferDistance);
	else
	    geometry = ls.buffer(bufferDistance, bufferSegments);
	return new EventLocation(geometry, this);			
    }

    
    
    public EventLocation fromString(String s) 
	throws WKTParseException	
    {
	try {
	    return new EventLocation(wktReader.read(s), this);
	}
	catch (com.vividsolutions.jts.io.ParseException e) {
	    throw new WKTParseException(e);
	}
    }

    private EventLocation fromSubEventsGeometries(Geometry[] subGeometries) {

	Geometry geometry = null;
	GeometryCollection gc = gFactory.createGeometryCollection(subGeometries);
	
	/*   // buffers deactivated for composite events...
	     if (bufferSegments == -1)
	     geometry = gc.convexHull().buffer(seBufferDistance);
	     else
	     geometry = gc.convexHull().buffer(seBufferDistance, bufferSegments);
	*/
	
	geometry = gc.convexHull();
	
	return new EventLocation(geometry, this);
    }      

    public EventLocation fromSubEvents(Collection<Event> subEvents) {
	
	Geometry[] subGeometries = new Geometry[subEvents.size()];
	Iterator<Event> iterator = subEvents.iterator();
	for (int i = 0; i < subGeometries.length && iterator.hasNext(); i++) 
	    subGeometries[i] = iterator.next().getLocation().geometry;

	return fromSubEventsGeometries(subGeometries);
    }


    public EventLocation fromSubEvents(Event[] subEvents) {
	
	Geometry[] subGeometries = new Geometry[subEvents.length];
	for (int i = 0; i < subGeometries.length; i++) 
	    subGeometries[i] = subEvents[i].getLocation().geometry;

	return fromSubEventsGeometries(subGeometries);
    }

    
    public EventLocation fromSubEventLocations(EventLocation[] locations) {
	
	Geometry geometry = null;
	Geometry[] subGeometries = new Geometry[locations.length];
	for (int i = 0; i < locations.length; i++)
	    subGeometries[i] = locations[i].geometry;
	GeometryCollection gc = gFactory.createGeometryCollection(subGeometries);
	
	/*   // buffers deactivated for composite events...
	     if (bufferSegments == -1)
	     geometry = gc.convexHull().buffer(seBufferDistance);
	     else
	     geometry = gc.convexHull().buffer(seBufferDistance, bufferSegments);
	*/
	
	geometry = gc.convexHull();

	return new EventLocation(geometry, this);
    }                                
}
