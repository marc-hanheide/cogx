/**
 * @author Marc Hanheide (marc@hanheide.de)
 */
package castutils.viewer.plugins;

import java.lang.reflect.Array;
import java.lang.reflect.Field;
import java.lang.reflect.Modifier;
import java.util.Map;
import java.util.Vector;
import java.util.Map.Entry;

import javax.management.ReflectionException;

import org.apache.log4j.Logger;

import Ice.ObjectImpl;
import castutils.XMLTag;

import com.thoughtworks.xstream.XStream;

/**
 * @author Marc Hanheide (marc@hanheide.de)
 * 
 */
public class ObjectImplInfo implements Plugin {

	private static final Class<?>[] LEAF_TYPES_ARR = new Class[] {
			Boolean.class, Float.class, Double.class, Integer.class,
			Long.class, String.class }; // add Collections.class?
	private boolean neverEscape = true;

	/*
	 * (non-Javadoc)
	 * 
	 * @see castutils.viewer.plugins.Plugin#toVector(Ice.ObjectImpl)
	 */
	@Override
	public Vector<Object> toVector(ObjectImpl iceObject) {
		Vector<Object> extraInfo = new Vector<Object>();
		extraInfo.add("<pre>" + toJSON(iceObject)+ "</pre>");
		//extraInfo.add("<pre>" + toXML(iceObject).toPlainString() + "</pre>");
		return extraInfo;
	}

	XMLTag toXML(Object o) {
		Class<?> cls = o.getClass();
		XMLTag rootTag = new XMLTag("object");
		rootTag.addAttr("type", cls.getName());

		// Get list of fields
		Field[] fields = cls.getFields(); // Inherited and 'public' fields
		// Field[] privateFields = c.getDeclaredFields(); // 'private' and
		// 'public' fields
		// field.setAccessible(true); // call on private fields before accessing

		// Transform all non-static fields to XML
		for (Field fld : fields) {
			if (isStatic(fld)) {
				// do nothing
				// getLogger().trace(
				// "Ignoring static field: " + fld.getName() + ":"
				// + fld.getType());
			} else {
				try {
					XMLTag child = toXMLSlave(fld, o);
					rootTag.addChild(child);
				} catch (ReflectionException ex) {
					getLogger().warn(
							"ReflectionException in WMLogger.toXMLSlave when processing Class:"
									+ cls.getCanonicalName() + " Field:"
									+ fld.getName()
									+ "\nlog entry may not be complete", ex);
				}
			}
		}

		return rootTag;
	}

	/**
	 * 
	 * @param fld
	 *            Field that is being transformed to XML
	 * @param owner
	 *            Object to which the field belongs
	 */
	private XMLTag toXMLSlave(Field fld, Object owner)
			throws ReflectionException {
		try {
			// Get field type and value
			Class<?> fldType = fld.getType();
			Object val = fld.get(owner);

			if (fldType.isArray()) {
				// Hand off to handleArray()
				// getLogger()
				// .debug(
				// "Found Array: " + fld.getName() + ":" + fldType
				// + " Length: "
				// + Array.getLength(fld.get(owner)));
				// rootTag.addChild(handleArray(fld, val));
				return handleArray(fld, val);
			} else if (Iterable.class.isAssignableFrom(fldType)) {
				// getLogger().debug(
				// "Found Iterable: " + fld.getName());
				return handleIterator(fld, val);
			} else if (Map.class.isAssignableFrom(fldType)) {
				// getLogger().debug(
				// "Found Iterable: " + fld.getName());
				return handleMap(fld, val);
			} else if (fldType.isEnum()) {
				// create an <enum> tag
				XMLTag enumTag = new XMLTag("enum");
				enumTag.addAttr("name", fld.getName());
				enumTag.addAttr("type", fldType.getName());
				if (val != null) {
					enumTag.addContents(val.toString(), false);
				}
				// rootTag.addChild(enumTag);
				return enumTag;

			} else if (isLeafType(fldType)) {
				// Create a leaf tag here
				// getLogger().debug(
				// "Found LeafType: " + fld.getName() + ":" + fldType);
				XMLTag child = getTagFor(fld.getType());
				child.addAttr("name", fld.getName());
				if (val != null) {
					child.addContents(val.toString());
				}
				// rootTag.addChild(child);
				return child;
			} else {
				// val is a complex object so recurse through toXML again
				// getLogger().debug(
				// "descending into " + fld.getName() + ":" + fldType);
				if (val != null) {
					XMLTag child = toXML(val); // This line could cause infinite
					// recursion if val has cyclic
					// references
					child.addAttr("name", fld.getName());
					// rootTag.addChild(child);
					return child;
				} else {
					XMLTag nullObj = new XMLTag("object");
					nullObj.addAttr("name", fld.getName());
					nullObj.addAttr("type", fld.getType().getName());
					return nullObj;
				}
			}
		} catch (IllegalArgumentException ex) {
			// getLogger().error("IllegalArgumentException in WMLogger.toXMLSlave()",
			// ex);
			throw new ReflectionException(ex);
		} catch (IllegalAccessException ex) {
			// getLogger().error("IllegalAccessException in WMLogger.toXMLSlave()",
			// ex);
			throw new ReflectionException(ex);
		}
	}

	/**
	 * 
	 * @returns true if class is in LEAF_TYPES_ARR or is a primitive
	 * @see #LEAF_TYPES_ARR
	 */
	private static boolean isLeafType(Class<?> c) {
		if (c.isPrimitive()) {
			return true;
		} else {
			for (Class<?> leaf_class : LEAF_TYPES_ARR) {
				if (c.equals(leaf_class)) {
					return true;
				}
			}
			return false;
		}
	}
	
	
	private String toJSON(Object o) {
		XStream xstream = new XStream();
		xstream.setMode(XStream.NO_REFERENCES);
		return escapeString(xstream.toXML(o));
		
//		GsonBuilder builder = new GsonBuilder();
//		Gson gson=builder.setPrettyPrinting().create();
//		return gson.toJson(o);
		
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
	public static String escapeString(String s) {
		for (String[] escape : XML_ESCAPES) {
			s = s.replaceAll(escape[0], escape[1]);
		}
		return s;
	}


	private XMLTag handleIterator(Field fld, Object arr) {
		XMLTag entry = new XMLTag("Iterable");
		entry.addAttr("name", fld.getName());
		entry.addAttr("type", fld.getGenericType().toString());

		Iterable<?> iter = (Iterable<?>) arr;
		int i = 0;
		for (Object val : iter) {
			Class<? extends Object> fldType = val.getClass();
			if (isLeafType(fldType)) {
				// Add a child tag
				// getLogger().debug(
				// "Found LeafType: " + fld.getName() + ":" + fldType);
				XMLTag child = getTagFor(fldType);
				child.addAttr("index", Integer.toString(i++));
				if (val != null) {
					child.addContents(val.toString());
				}
				entry.addChild(child);
			} else {
				// getLogger().debug(
				// "descending into " + fld.getName() + ":" + fldType);
				XMLTag child = toXML(val);
				child.addAttr("index", Integer.toString(i));
				entry.addChild(child);
			}
		}

		return entry;
	}

	private XMLTag handleMap(Field fld, Object arr) {
		XMLTag entry = new XMLTag("Iterable");
		entry.addAttr("name", fld.getName());
		entry.addAttr("type", fld.getGenericType().toString());

		Map<?, ?> iter = (Map<?, ?>) arr;

		for (Entry<?, ?> val : iter.entrySet()) {
			Class<? extends Object> fldType = val.getValue().getClass();
			if (isLeafType(fldType)) {
				// Add a child tag
				// getLogger().debug(
				// "Found LeafType: " + fld.getName() + ":" + fldType);
				XMLTag child = getTagFor(fldType);
				child.addAttr("key", val.getKey().toString());
				if (val.getValue() != null) {
					child.addContents(val.getValue().toString());
				}
				entry.addChild(child);
			} else {
				// getLogger().debug(
				// "descending into " + fld.getName() + ":" + fldType);
				XMLTag child = toXML(val.getValue());
				child.addAttr("key", val.getKey().toString());
				entry.addChild(child);
			}
		}

		return entry;
	}

	/**
	 * Creates an &lt;array&gt; tag that represents the array it is given
	 * 
	 * @param fld
	 * @param arr
	 *            Must be an array object
	 * @return
	 */
	private XMLTag handleArray(Field fld, Object arr) {
		int len = Array.getLength(arr);
		// getLogger().debug(
		// "handleArray: " + fld.getName() + " length:"
		// + Integer.toString(len));

		XMLTag entry = new XMLTag("array");
		entry.addAttr("name", fld.getName());
		entry.addAttr("length", Integer.toString(len));
		entry.addAttr("type", fld.getType().getComponentType().getName());

		// Output each element as a child
		for (int i = 0; i < len; i++) {
			Class<?> fldType = fld.getType().getComponentType();
			Object val = Array.get(arr, i);
			if (isLeafType(fldType)) {
				// Add a child tag
				// getLogger().debug(
				// "Found LeafType: " + fld.getName() + ":" + fldType);
				XMLTag child = getTagFor(fld.getType().getComponentType());
				child.addAttr("index", Integer.toString(i));
				if (val != null) {
					child.addContents(val.toString());
				}
				entry.addChild(child);
			} else {
				// getLogger().debug(
				// "descending into " + fld.getName() + ":" + fldType);
				XMLTag child = toXML(val);
				child.addAttr("index", Integer.toString(i));
				entry.addChild(child);
			}
		}

		return entry;
	}

	/**
	 * <B>Warning:</B> should only be used on leafType classes
	 * 
	 * @param cls
	 *            the class to be represented
	 * @returns a XMLTag for representing the requrired class
	 */
	private XMLTag getTagFor(Class<?> cls) {
		String s = cls.getSimpleName().toLowerCase();
		XMLTag t = new XMLTag(s);
		t.setUseCDATAEscaping(needsEscaping(cls));
		return t;
	}

	/**
	 * @returns true for leafType classes whose values should be CDATA escaped
	 *          in XML
	 */
	private boolean needsEscaping(Class<?> cls) {
		if (neverEscape)
			return false;
		return !(cls.isPrimitive() || cls == Float.class || cls == Double.class
				|| cls == Integer.class || cls == Long.class);
	}

	private static Logger getLogger() {
		return Logger.getLogger(ObjectImplInfo.class);
	}

	private static boolean isStatic(Field f) {
		return ((f.getModifiers() & Modifier.STATIC) == Modifier.STATIC);
	}

}
