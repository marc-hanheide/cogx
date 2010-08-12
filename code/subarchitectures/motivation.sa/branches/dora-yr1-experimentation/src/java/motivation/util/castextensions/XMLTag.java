package motivation.util.castextensions;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

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
    private static final String INDENT = "\t";

    public XMLTag(String name) {
        this.name = name;
    }

    public XMLTag(String name, Map<String, String> attrs) {
        this.name = name;
        this.attrs = attrs;
    }

    /** <B> WARNING:</B> Make sure keys are valid XML attribute names
     */
    public void addAttr(String key, String val) {
        attrs.put(key, val);
    }

    public void addChild(XMLTag child) {
        if (contents != null) {
            throw new IllegalStateException("XMLTag cannot have childen and contents string");
        }
        children.add(child);
    }
//        <![CDATA[ and ends with ]]>
    private static final String CDATA_START = "<![CDATA[";
    private static final String CDATA_END = "]]>";

    /**
     * Wraps a string in CDATA tags making sure there is no unescaped instance
     * of ']]>' inside the string.
     * @param s
     * @return
     */
    public static String escapeCdata(String s) {
        s = s.replaceAll(CDATA_END, CDATA_END + CDATA_END + CDATA_START);// Escape all ']]>' in the string
        return CDATA_START + s + CDATA_END;
    }

    public void addContents(String contents) {
        if (!"".equals(contents)) {
            addContents(contents, useCDATAEscaping);
        }
    }

    public void addContents(String contents, boolean cdataEscape) {
        if (!children.isEmpty()) {
            throw new IllegalStateException("XMLTag cannot have childen and contents string");
        }

        if (cdataEscape) {
            this.contents = escapeCdata(contents);
        } else {
            this.contents = contents;
        }
    }
    ////
    /** list of the illegal xml chars and their replacements */
    private static final String[][] XML_ESCAPES = {
        {"&", "&amp;"}, // Ampersand escape should stay before all others
        {"<", "&lt;"},
        {">", "&gt;"},
        {"\"", "&quot;"},
        {"'", "&apos;"}
    };

    /**
     * Escapes a string for XML
     * @param s string to be escaped
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
     * @param s string to be escaped
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
     * Converts the XMLTags into a string representation,
     * making sure indentation is correct.
     * @return String formatted XML
     */
    @Override
    public String toString() {
        return toStringHelper("");
    }

    /**
     * Recursive helper for toString() that handles the indentation.
     * @param indent prefix for each line
     * @return
     * @see #toString()
     */
    private String toStringHelper(String indent) {
        // Start tag
        StringBuilder sb = new StringBuilder(indent).append("<").append(name);

        // Output Attributes
        if (!attrs.isEmpty()) {
            for (String key : attrs.keySet()) {
                sb.append(" ").append(key).append("=\"").append(escapeString(attrs.get(key))).append("\"");
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
}
