/**
 * 
 */
package castutils.castextensions.wmeditor.serializer;

import java.io.Reader;

import com.thoughtworks.xstream.XStream;

/**
 * @author cogx
 * 
 */
public class XMLSerializer implements Serializer {

	final private XStream xstream;

	public XMLSerializer() {
		xstream = new XStream();
	}

	/*
	 * (non-Javadoc)
	 * 
	 * @see castutils.castextensions.wmeditor.Serializer#dump(Ice.Object)
	 */
	@Override
	public String dump(Object obj) {
		return xstream.toXML(obj);
	}

	/*
	 * (non-Javadoc)
	 * 
	 * @see castutils.castextensions.wmeditor.Serializer#load(java.lang.String)
	 */
	@Override
	public Object load(String str) {
		return xstream.fromXML(str);
	}

	@Override
	public Object load(Reader reader) {
		return xstream.fromXML(reader);
	}

}
