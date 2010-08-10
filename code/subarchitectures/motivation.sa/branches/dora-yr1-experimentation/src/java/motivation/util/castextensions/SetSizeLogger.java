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
            String logString = "<SETSIZE type=\""+specClass.getName()+"\" cast_time=\""+WMLogger.CASTTimeToString(_wmc.timestamp)+"\">"+addresses.size()+"</SETSIZE>";
            getLogger().info(logString);
            println(logString);
        }
    }
}
