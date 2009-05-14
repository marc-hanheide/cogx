/**
 * 
 */
package balt.core.data;

import java.util.Hashtable;

/**
 * Produce typenames that match c++ equivalents to support translation.
 * 
 * @author nah
 * 
 */
public class BALTType {

	private final static Hashtable<Class<?>, String> m_typeMap;
	private final static Hashtable<String, Class<?>> m_classMap;

	static {
		m_typeMap = new Hashtable<Class<?>, String>();
		m_classMap = new Hashtable<String, Class<?>>();

		m_classMap.put("bool", Boolean.class);
		m_classMap.put("int", Integer.class);
		m_classMap.put("string", String.class);
	}

	public final static <Type> String typeName(Type _obj) {
		return typeName(_obj.getClass());
	}

	public final static <Type> String typeName(Class<Type> _cls) {

		String type = m_typeMap.get(_cls);
		if (type != null) {
			return type;
		}
		type = demangle(_cls);
		assert (type != null && type.length() > 0);
		m_typeMap.put(_cls, type);
		m_classMap.put(type, _cls);
		return type;
	}

	private final static <Type> String demangle(Class<Type> _cls) {

		if (_cls.equals(String.class)) {
			return "string";
		}

		String name = _cls.getName();
		name = name.replaceAll("\\.", "::");

		// and some quick hacks to align types across different namespaces
		name = name.replaceAll("core::", "");
		name = name.replaceAll("data::", "");

		// and to refactor array names
		// remove first 2 changes
		if (name.startsWith("[L")) {
			name = name.replaceAll("\\[L", "");
			name = name.replaceAll(";", "List");
		}

		// System.out.println("BALTType.demangle(): " + name);

		return name;
	}

	public static Class<?> classFromType(String _type) {
		Class<?> cls = m_classMap.get(_type);

		if (cls == null) {
			cls = desparatelyTryToRecover(_type);
		}

		if (cls == null) {
			// ugly for the time being
			throw new RuntimeException("Do not have class for: " + _type);
		}

		return cls;

	}

	private static Class<?> desparatelyTryToRecover(String _type) {
		// reverse sub
		String className = _type.replace("::", ".");
		if (className.endsWith("List")) {
			className = "\\[L" + className + ";";
		}

		System.out.println("BALTType.desparatelyTryToRecover(): " + _type
				+ " -> " + className);

		try {
			return Class.forName(className);
		}
		catch (ClassNotFoundException e) {
			e.printStackTrace();
			return null;
		}
		
	}

}
