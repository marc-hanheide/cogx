/** _
 *  part of the ELM system.
 *  @author Dennis Stachowicz
 */

package elm.event;

import com.vividsolutions.jts.geom.Geometry;

public class EventLocation {

	protected Geometry geometry 		= null;
        protected EventLocationFactory factory  = null;

	// use EventLocationFactory objects for construction
        protected EventLocation(Geometry geometry, EventLocationFactory factory) {
		this.geometry = geometry;
		this.factory  = factory;
	}
        public EventLocationFactory getFactory() {
	        return factory;
	}
	public int getSRID() {
		return factory.getSRID();
	}
        public String getWKTString() {
        	return geometry.toText();
	}        
	public String getPostGISString() {
		return "GeomFromText('" + getWKTString() + "', " + getSRID() + ")";
	}
        public String toString() {
        	return getWKTString();
	}
}
