/**
 * @author Marc Hanheide (marc@hanheide.de)
 */
package castutils.castextensions;

import nu.xom.Builder;
import nu.xom.Document;
import nu.xom.Element;

import com.thoughtworks.xstream.XStream;
import com.thoughtworks.xstream.io.xml.XomWriter;

/**
 * @author Marc Hanheide (marc@hanheide.de)
 * 
 */
public final class IceXMLSerializer {
	final static XStream xstream = new XStream();
	final static Builder db = new nu.xom.Builder();


	static {
		
	}

	public static String toXMLString(Object o) {
		return xstream.toXML(o);
	}

	public static void omitField(Class definedIn, String fieldName) {
		xstream.omitField(definedIn, fieldName);
	}

	public static <T extends Ice.Object> T fromXMLString(String in, Class<T> type) {
		return type.cast(xstream.fromXML(in));
	}
	
	public static Document toXomDom(Ice.Object o) {
		Element root=new Element("test");
		xstream.marshal(o, new XomWriter(root));
		Document doc = new Document((Element) root.getChildElements().get(0).copy());
		return doc;
	}
	
}
