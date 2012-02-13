/**
 * 
 */
package motivation.util.castextensions;

import java.util.LinkedList;
import java.util.Map;
import java.util.Set;


import Ice.ObjectImpl;
import cast.CASTException;
import cast.architecture.ChangeFilterFactory;
import cast.architecture.ManagedComponent;
import cast.architecture.WorkingMemoryChangeReceiver;
import cast.cdl.WorkingMemoryAddress;
import cast.cdl.WorkingMemoryChange;
import cast.cdl.WorkingMemoryOperation;
import java.util.HashSet;

/**
 * @author marc
 * 
 */
public class SetSizeLogger extends ManagedComponent {

    LinkedList<Class> SUBSCRIBED_CLASSES = new LinkedList<Class>();

    @Override
    public void start() /* throws UnknownSubarchitectureException */ {
        // create log
        getLogger().info("SetSizeLogger Started");
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
                println("Subscribtion: " + className);
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
        getLogger().info("register listeners");
        SetSizeChangeReceiver sscr = new SetSizeChangeReceiver(type);
        addChangeFilter(ChangeFilterFactory.createTypeFilter(type,
                WorkingMemoryOperation.ADD), sscr);
        addChangeFilter(ChangeFilterFactory.createTypeFilter(type,
                WorkingMemoryOperation.DELETE), sscr);
        addChangeFilter(ChangeFilterFactory.createTypeFilter(type,
                WorkingMemoryOperation.OVERWRITE), sscr);
    }

    public class SetSizeChangeReceiver<T extends Ice.ObjectImpl> implements WorkingMemoryChangeReceiver {

        /** The type of object this ChangeReciever is interested in */
        Class<T> specClass;
        /** Has to be a Set&lt;String&gt; as WorkingMemoryAddress doesn't 
         * correctly implement .equals or .hashCode() */
//        Set<String> addresses = new HashSet<String>();
        Set<WorkingMemoryAddress> addresses = new HashSet();

        public SetSizeChangeReceiver(Class<T> specClass) {
            super();
            this.specClass = specClass;
        }

        @SuppressWarnings("unchecked")
        @Override
        public synchronized void workingMemoryChanged(WorkingMemoryChange _wmc) throws CASTException {


            String wmas = WMLogger.WMAToString(_wmc.address);
            WorkingMemoryAddress wma = _wmc.address;
            switch (_wmc.operation) {
                case ADD:
                    addresses.add(wma);
                    if (!addresses.contains(wma)) {
                        throw new IllegalStateException("Object was not correctly added to set");
                    }
                    getLogger().debug("new " + specClass + " in SetSizeLogger size: " + addresses.size());
                    break;
                case OVERWRITE:
                    if (!addresses.contains(wma)) {
                        getLogger().fatal("error recieving overwrite for " + specClass + " in SetSizeLogger");
                        throw new IllegalStateException(wmas + " is missing from the set");
                    }
                    getLogger().debug("existing " + specClass + " in SetSizeLogger size: " + addresses.size());
                    break;
                case DELETE:
                    boolean success = addresses.remove(wma);

                    if (success) {
                        getLogger().debug("deleted " + specClass + " in SetSizeLogger");
                        getLogger().debug("size: " + addresses.size());
                    } else {
                        getLogger().fatal("error deleting " + specClass + " in SetSizeLogger");
                        throw new IllegalStateException(wmas + " is missing from the set");
                    }
                    break;
                default:
                    getLogger().warn("WMChange operation ignored in SetSizeLogger: " + _wmc.operation);
                    break;
            }
            // output the set size to logs
            XMLTag t = new XMLTag("SETSIZE");
            t.addAttr("type", specClass.getName());
            t.addCastTimeAttr(_wmc.timestamp);
            t.addContents(addresses.size());
            String logString = t.toString();
            getLogger().info(logString);
            println(logString);
        }
    }
}