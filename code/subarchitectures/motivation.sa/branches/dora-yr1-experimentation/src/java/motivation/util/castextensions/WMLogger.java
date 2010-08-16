/**
 * 
 */
package motivation.util.castextensions;

import cast.core.logging.ComponentLogger;
import java.util.Collection;
import java.util.Collections;
import java.util.HashMap;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;
import java.util.Set;

import org.apache.log4j.Level;
import org.apache.log4j.Logger;

import Ice.ObjectImpl;
import SpatialData.Place;
import SpatialData.PlaceStatus;
import cast.AlreadyExistsOnWMException;
import cast.CASTException;
import cast.DoesNotExistOnWMException;
import cast.PermissionException;
import cast.UnknownSubarchitectureException;
import cast.WMException;
import cast.architecture.ChangeFilterFactory;
import cast.architecture.ManagedComponent;
import cast.architecture.WorkingMemoryChangeReceiver;
import cast.cdl.CASTTime;
import cast.cdl.WorkingMemoryAddress;
import cast.cdl.WorkingMemoryChange;
import cast.cdl.WorkingMemoryOperation;
import cast.cdl.WorkingMemoryPermissions;
import cast.core.CASTData;
import java.lang.reflect.Array;
import java.lang.reflect.Field;
import java.lang.reflect.Modifier;
import java.util.ArrayList;

/**
 * @author marc
 * 
 */
public class WMLogger extends ManagedComponent {

    LinkedList<Class> SUBSCRIBED_CLASSES = new LinkedList<Class>();
    
    private static final Class[] LEAF_TYPES_ARR = new Class[]{
        Boolean.class,
        Float.class, Double.class,
        Integer.class, Long.class,
        String.class}; //add Collections.class?

    /**
     *
     * @returns true if class is in LEAF_TYPES_ARR or is a primitive
     * @see #LEAF_TYPES_ARR
     */
    private static boolean isLeafType(Class<?> c) {
        if (c.isPrimitive()) {
            return true;
        } else {
            for (Class leaf_class : LEAF_TYPES_ARR) {
                if (c.equals(leaf_class)) {
                    return true;
                }
            }
            return false;
        }
    }

    /**
     * @returns true for leafType classes whose values should be CDATA escaped in XML
     */
    private boolean needsEscaping(Class cls) {
        return !(cls.isPrimitive() || cls == Float.class || cls == Double.class
                || cls == Integer.class || cls == Long.class);
    }

    /**
     * <B>Warning:</B> should only be used on leafType classes
     * @param cls the class to be represented
     * @returns a XMLTag for representing the requrired class
     */
    private XMLTag getTagFor(Class cls) {
        String s = cls.getSimpleName().toLowerCase();
        XMLTag t = new XMLTag(s);
        t.useCDATAEscaping = needsEscaping(cls);
        return t;
    }

    private static boolean isStatic(Field f) {
        return ((f.getModifiers() & Modifier.STATIC) == Modifier.STATIC);
    }

    /**
     * Creates an &lt;array&gt; tag that represents the array it is given
     * @param fld
     * @param arr Must be an array object
     * @return
     */
    private XMLTag handleArray(Field fld, Object arr) {
        int len = Array.getLength(arr);
        getLogger().debug("handleArray: " + fld.getName() + " length:" + Integer.toString(len));

        XMLTag entry = new XMLTag("array");
        entry.addAttr("name", fld.getName());
        entry.addAttr("length", Integer.toString(len));
        entry.addAttr("type", fld.getType().getComponentType().getName());



        // Output each element as a child
        for (int i = 0; i < len; i++) {
            Class fldType = fld.getType().getComponentType();
            Object val = Array.get(arr, i);
            if (isLeafType(fldType)) {
                // Add a child tag
                getLogger().debug("Found LeafType: " + fld.getName() + ":" + fldType);
                XMLTag child = getTagFor(fld.getType().getComponentType());
                child.addAttr("index", Integer.toString(i));
                if (val != null) {
                    child.addContents(val.toString());
                }
                entry.addChild(child);
            } else {
                getLogger().debug("descending into " + fld.getName() + ":" + fldType);
                XMLTag child = toXML(val);
                child.addAttr("index", Integer.toString(i));
                entry.addChild(child);
            }
        }

        return entry;
    }

    /**
     *
     * @param fld Field that is being transformed to XML
     * @param owner Object to which the field belongs
     * @param rootTag XML representation so far
     */
    private void toXMLSlave(Field fld, Object owner, XMLTag rootTag) {
        try {
            // Get field type and value
            Class fldType = fld.getType();
            Object val = fld.get(owner);

            if (fldType.isArray()) {
                // Hand off to handleArray()
                getLogger().debug("Found Array: " + fld.getName() + ":" + fldType + " Length: " + Array.getLength(fld.get(owner)));
                rootTag.addChild(handleArray(fld, val));
            } else if (fldType.isEnum()) {
                // create an <enum> tag
                XMLTag enumTag = new XMLTag("enum");
                enumTag.addAttr("name", fld.getName());
                enumTag.addAttr("type", fldType.getName());
                if (val != null) {
                    enumTag.addContents(val.toString(), false);
                }

            } else if (isLeafType(fldType)) {
                // Create a leaf tag here
                getLogger().debug("Found LeafType: " + fld.getName() + ":" + fldType);
                XMLTag child = getTagFor(fld.getType());
                child.addAttr("name", fld.getName());
                if (val != null) {
                    child.addContents(val.toString());
                }
                rootTag.addChild(child);
            } else {
                // val is a complex object so recurse through toXML again
                getLogger().debug("descending into " + fld.getName() + ":" + fldType);
                if (val != null) {
                    XMLTag child = toXML(val); // This line could cause infinite recursion if val has cyclic references
                    child.addAttr("name", fld.getName());
                    rootTag.addChild(child);
                }
            }
        } catch (IllegalArgumentException ex) {
            getLogger().error("Error in WMLogger.toXMLSlave()", ex);
        } catch (IllegalAccessException ex) {
            getLogger().error("Error in WMLogger.toXMLSlave()", ex);
        }
    }

    /**
     * Takes an object and returns an XML representation of the object and all
     * its public non-static fields.
     * @param owner
     * @return
     */
    public XMLTag toXML(Object o) {
        Class cls = o.getClass();
        XMLTag rootTag = new XMLTag("object");
        rootTag.addAttr("type", cls.getName());

        // Get list of fields
        Field[] fields = cls.getFields(); // Inherited and 'public' fields
//        Field[] privateFields = c.getDeclaredFields(); // 'private' and 'public' fields
//        field.setAccessible(true); // call on private fields before accessing


        // Transform all non-static fields to XML
        for (Field fld : fields) {
            if (isStatic(fld)) {
                // do nothing
                getLogger().debug("Ignoring static field: " + fld.getName() + ":" + fld.getType());
            } else {
                toXMLSlave(fld, o, rootTag);
            }
        }

        return rootTag;
    }

    @Override
    public void start() /* throws UnknownSubarchitectureException */ {
        // create log
        getLogger().debug("WMLogger Started");
        // register listeners
        for (Class cl : SUBSCRIBED_CLASSES) {
            register(cl);
        }
    }

    @Override
    protected void configure(Map<String, String> map) {
        super.configure(map);

        // Get subscriptions
        String subs = map.get("--subscribe");
        parseSubscriptions(subs);
    }

    /**
     * Takes a list of classes as a string and looks up and loads them.
     * They are added to the list SUBSCRIBED_CLASSES and in the start() method
     * every class in this list has a listener added for it.
     * @param subs comma seperated list of classes to listen for on working memory
     * @see #start()
     * @see #SUBSCRIBED_CLASSES
     */
    private void parseSubscriptions(String subs) {
        String[] subsArr = subs.split("\\s*,\\s*"); // regex = [whitespace][comma][whitespace]
        for (String className : subsArr) {
            try {
                getLogger().debug("Subscribtion: " + className);
                ClassLoader.getSystemClassLoader().loadClass(className);
                Class<? extends ObjectImpl> subscription = (Class<? extends ObjectImpl>) Class.forName(className); //TODO: Can this cast be tidier?
                SUBSCRIBED_CLASSES.add(subscription);
            } catch (ClassNotFoundException e) {
                String error = "trying to register for a class that doesn't exist. [" + className + "]";
                getLogger().error(error);  // log4j
                println(error);
                e.printStackTrace();
            }
        }
    }

    public <T extends Ice.ObjectImpl> void register(Class<T> type) {
        getLogger().trace("register listeners");
        addChangeFilter(ChangeFilterFactory.createTypeFilter(type,
                WorkingMemoryOperation.ADD), new LoggingChangeReceiver(type));
        addChangeFilter(ChangeFilterFactory.createTypeFilter(type,
                WorkingMemoryOperation.DELETE), new LoggingChangeReceiver(type));
        addChangeFilter(ChangeFilterFactory.createTypeFilter(type,
                WorkingMemoryOperation.OVERWRITE), new LoggingChangeReceiver(type));
    }

    public class LoggingChangeReceiver<T extends Ice.ObjectImpl> implements WorkingMemoryChangeReceiver {

        /** The type of object this ChangeReciever is interested in */
        Class<T> specClass;

        public LoggingChangeReceiver(Class<T> specClass) {
            super();
            this.specClass = specClass;
        }

        @SuppressWarnings("unchecked")
        @Override
        public synchronized void workingMemoryChanged(WorkingMemoryChange _wmc) throws CASTException {

            T m;
            final String wma = WMAToString(_wmc.address);
            XMLTag rootTag = new XMLTag("WMOPERATION");
            rootTag.addAttr("type", _wmc.operation.toString());
            rootTag.addAttr("address", wma);
            rootTag.addCastTimeAttr(_wmc.timestamp);
            rootTag.addAttr("src", _wmc.src);


            switch (_wmc.operation) {
                case ADD:
                case OVERWRITE:
                    try {
                        m = getMemoryEntry(_wmc.address, specClass);
                    } catch (DoesNotExistOnWMException e) {
                        // If we catch this the entry was deleted before we could load and output its data
                        WMException wmex = new WMException("WMEntry deleted before it could be loaded in WMLogger.workingMemoryChanged", _wmc.address);
                        wmex.initCause(e);
                        throw e;
                    }
                    rootTag.addChild(toXML(m));
                    rootTag.addAttr("content_type", m.getClass().getName());
                    break;

                case DELETE:
                    rootTag.addAttr("content_type", specClass.getName());
                    break;

                default:
                    getLogger().warn("WMChange operation ignored in WMLogger.: " + _wmc.operation);
                    break;
            }
            final String tagString = rootTag.toString();
            println("printTagString:\n" + tagString);
            getLogger().info(tagString);

        }
    }

    public static String WMAToString(WorkingMemoryAddress wma) {
        return wma.id + "@" + wma.subarchitecture;
    }

    public static String CASTTimeToString(CASTTime ct) {
        return Long.toString((ct.s * 1000) + (ct.us / 1000));
//        return Double.toString((ct.s * 1000) + (ct.us /(double) 1000)); // More accurate, if required
    }
}


