package castutils;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import cast.cdl.CASTTime;

/**
 * @author Ben Page
 */
public class XMLTag {

	private final String name;
/** <B> WARNING:</B> If manipulating the map yourself, make sure keys are
     * valid XML attribute names; ie. no spaces, '<', '>' or '&'.
     */
	Map<String, String> attrs = new HashMap<String, String>();
	List<XMLTag> children = new ArrayList<XMLTag>();
	private String contents;
	boolean useCDATAEscaping = true;
	private static final String INDENT = "  ";

	public XMLTag(String name) {
		this.name = name;
	}

	public XMLTag(String name, Map<String, String> attrs) {
		this.name = name;
		this.attrs = attrs;
	}

	/**
	 * <B> WARNING:</B> Make sure keys are valid XML attribute names
	 */
	public void addAttr(String key, String val) {
		attrs.put(key, val);
	}

	/**
	 * Convenience method for adding a <tt>cast_time</tt> attribute.
	 * 
	 * @param time
	 * @see #addAttr(java.lang.String, java.lang.String)
	 */
	public void addCastTimeAttr(CASTTime time) {
		addAttr("cast_time", WMLogger.CASTTimeToString(time));
	}

	public void addChild(XMLTag child) {
		if (contents != null) {
			throw new IllegalStateException(
					"XMLTag cannot have childen and contents string");
		}
		children.add(child);
	}

	// <![CDATA[ and ends with ]]>
	private static final String CDATA_START = "<![CDATA[";
	private static final String CDATA_END = "]]>";

	/**
	 * Wraps a string in CDATA tags making sure there is no unescaped instance
	 * of ']]>' inside the string.
	 * 
	 * @param s
	 * @return
	 */
	public static String escapeCdata(String s) {
		s = s.replaceAll(CDATA_END, CDATA_END + CDATA_END + CDATA_START);// Escape
		// all
		// ']]>'
		// in
		// the
		// string
		return CDATA_START + s + CDATA_END;
	}

	/**
	 * Adds the string as contents to the XML tag, a tag cannot have contents
	 * and children. Escaping defaults to useCDATAEscaping
	 * 
	 * @param bool_contents
	 *            contents of the tag
	 * @see #addContents(java.lang.String, boolean)
	 * @see #useCDATAEscaping
	 */
	public void addContents(String contents) {
		addContents(contents, useCDATAEscaping);

	}

	public void addContents(Number contents) {
		addContents(contents.toString(), false);
	}

	public void addContents(boolean bool_contents) {
		addContents(Boolean.toString(bool_contents), false);
	}

	/**
	 * Adds the string as contents to the XML tag, a tag cannot have contents
	 * and children.
	 * 
	 * @param bool_contents
	 *            contents of the tag
	 * @param cdataEscape
	 *            should the data be wrapped in CDATA escaping
	 * @see #addContents(java.lang.String)
	 */
	public void addContents(String contents, boolean cdataEscape) {
		if (!children.isEmpty()) {
			throw new IllegalStateException(
					"XMLTag cannot have childen and contents string");
		}

		if (cdataEscape) {
			this.contents = escapeCdata(contents);
		} else {
			this.contents = contents;
		}
		
	}

	// //
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
	public static String escapeString(String s) {
		for (String[] escape : XML_ESCAPES) {
			s = s.replaceAll(escape[0], escape[1]);
		}
		return s;
	}

	/**
	 * Unescapes a string for XML
	 * 
	 * @param s
	 *            string to be escaped
	 * @see #XML_ESCAPES
	 * @return a string with &amp; etc removed
	 */
	public static String unescapeString(String s) {

		for (int i = XML_ESCAPES.length - 1; i >= 0; i--) {
			String[] escape = XML_ESCAPES[i];
			s = s.replaceAll(escape[1], escape[0]);
		}
		return s;
	}

	/**
	 * Converts the XMLTags into a string representation, making sure
	 * indentation is correct.
	 * 
	 * @return String formatted XML
	 */
	@Override
	public String toString() {
		// Log4J CDATA work around -- Log4j wraps messages in CDATA tags, we
		// don't want this
		// return CDATA_END + toStringHelper("") + CDATA_START; // The
		// workaround doesn't work, log4j escapes the escaping
		return toStringHelper("");
	}

	/**
	 * Converts the XMLTags into a string representation, making sure
	 * indentation is correct.
	 * 
	 * @return String formatted XML
	 */
	public String toPlainString() {
		// Log4J CDATA work around -- Log4j wraps messages in CDATA tags, we
		// don't want this
		// return CDATA_END + toStringHelper("") + CDATA_START; // The
		// workaround doesn't work, log4j escapes the escaping
		return toPlainStringHelper("  ");
	}

	/**
	 * Recursive helper for toString() that handles the indentation.
	 * 
	 * @param indent
	 *            prefix for each line
	 * @return
	 * @see #toString()
	 */
	private String toStringHelper(String indent) {
		// Start tag
		StringBuilder sb = new StringBuilder(indent).append("<").append(name);

		// Output Attributes
		if (!attrs.isEmpty()) {
			for (String key : attrs.keySet()) {
				sb.append(" ").append(key).append("=\"").append(
						escapeString(attrs.get(key))).append("\"");
			}
		}

		if (children.isEmpty()) {
			if (contents == null) {
				// Self closing tag
				sb.append("/>");
			} else {
				sb.append('>');
				sb.append(contents);
				sb.append("</").append(name).append(">");
			}
		} else {
			// Output children and close tag
			sb.append(">\n");
			for (XMLTag tag : children) {
				sb.append(tag.toStringHelper(indent + INDENT)).append("\n");
			}
			// Close tag
			sb.append(indent).append("</").append(name).append(">");
		}

		return sb.toString();
	}

	/**
	 * Recursive helper for toString() that handles the indentation.
	 * 
	 * @param indent
	 *            prefix for each line
	 * @return
	 * @see #toString()
	 */
	private String toPlainStringHelper(String indent) {
		// Start tag
		StringBuilder sb = new StringBuilder(indent).append("(").append(name)
				.append("[");

		// Output Attributes
		if (!attrs.isEmpty()) {
			for (String key : attrs.keySet()) {
				sb.append(" ").append(key).append("=\"").append(
						escapeString(attrs.get(key))).append("\"");
			}
		}

		sb.append("]");

		if (children.isEmpty()) {
			if (contents == null) {
			} else {
				sb.append(" => ");
				sb.append(contents);
			}
			sb.append(")<br/>");
		} else {
			sb.append("<br/>");
			// Output children and close tag
			for (XMLTag tag : children) {
				sb.append(tag.toPlainStringHelper(indent + INDENT));
			}
			sb.append(indent).append(")<br/>");
		}

		return sb.toString();
	}

	/**
	 * @return the useCDATAEscaping
	 */
	public boolean isUseCDATAEscaping() {
		return useCDATAEscaping;
	}

	/**
	 * @param useCDATAEscaping
	 *            the useCDATAEscaping to set
	 */
	public void setUseCDATAEscaping(boolean useCDATAEscaping) {
		this.useCDATAEscaping = useCDATAEscaping;
	}
}
