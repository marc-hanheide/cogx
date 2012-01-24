/**
 * @author Marc Hanheide (marc@hanheide.de)
 */
package castutils.viewer.plugins;

import java.util.Vector;

import Ice.ObjectImpl;
import castutils.castextensions.IceXMLSerializer;

/**
 * @author Marc Hanheide (marc@hanheide.de)
 * 
 */
public class DefaultXMLInfo implements Plugin {

	/*
	 * (non-Javadoc)
	 * 
	 * @see castutils.viewer.plugins.Plugin#toVector(Ice.ObjectImpl)
	 */
	@Override
	public Vector<Object> toVector(ObjectImpl iceObject) {
		Vector<Object> extraInfo = new Vector<Object>();
		extraInfo.add("<pre>" + toXML(iceObject)+ "</pre>");
		//extraInfo.add("<pre>" + toXML(iceObject).toPlainString() + "</pre>");
		return extraInfo;
	}
	
	public static String toXML(Object o) {
		return escapeString(IceXMLSerializer.toXMLString(o));
	}

	/** list of the illegal xml chars and their replacements */
	private static final String[][] XML_ESCAPES = {
			{ "&", "&amp;" }, // Ampersand escape should stay before all others
			{ "<", "&lt;" }, { ">", "&gt;" }, { "\"", "&quot;" },
			{ "'", "&apos;" } };

	/**
	 * Escapes a string for XML
	 * 
	 * @param s
	 *            string to be escaped
	 * @see #XML_ESCAPES
	 * @return a XML safe string
	 */
	static String escapeString(String s) {
		for (String[] escape : XML_ESCAPES) {
			s = s.replaceAll(escape[0], escape[1]);
		}
		return s;
	}

}
