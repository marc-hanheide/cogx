/**
 * 
 */
package motivation.util.castextensions;

import cast.cdl.WorkingMemoryChangeFilter;
import java.util.Collection;
import java.util.Iterator;
import java.util.Set;


import Ice.ObjectImpl;
import SpatialData.Place;
import cast.CASTException;
import cast.architecture.ChangeFilterFactory;
import cast.architecture.ManagedComponent;
import cast.architecture.WorkingMemoryChangeReceiver;
import cast.cdl.WorkingMemoryAddress;
import cast.cdl.WorkingMemoryChange;
import cast.cdl.WorkingMemoryOperation;
import java.util.HashSet;
import motivation.slice.Motive;

/**
 * @author marc
 * 
 */
public class PlaceSetSizeLogger extends ManagedComponent {

    static final String PLACE_CLASS_NAME = "SpatialData.Place";

    @Override
    public void start() /* throws UnknownSubarchitectureException */ {
        // create log
        getLogger().info("PlaceSetSizeLogger Started");
        // register listener
        register();
    }

    public <T extends Ice.ObjectImpl> void register() {
        try {
//            Class<? extends ObjectImpl> type = (Class<? extends ObjectImpl>) Class.forName(PLACE_CLASS_NAME);
            Class<? extends ObjectImpl> type = (Class.forName(PLACE_CLASS_NAME)).asSubclass(ObjectImpl.class);
            getLogger().info("register listener");
            MotiveChangeReciever sscr = new MotiveChangeReciever();
            addChangeFilter(ChangeFilterFactory.createTypeFilter(type, WorkingMemoryOperation.ADD), sscr);
            addChangeFilter(ChangeFilterFactory.createTypeFilter(type, WorkingMemoryOperation.DELETE), sscr);
            addChangeFilter(ChangeFilterFactory.createTypeFilter(type, WorkingMemoryOperation.OVERWRITE), sscr);
        } catch (ClassNotFoundException ex) {
            getLogger().fatal("couldn't load motive class in MotiveSetSizeLogger.register", ex);
        }
    }
    Set<WorkingMemoryAddress> trueplace = new HashSet<WorkingMemoryAddress>();
    Set<WorkingMemoryAddress> placeholder = new HashSet<WorkingMemoryAddress>();

    public class MotiveChangeReciever implements WorkingMemoryChangeReceiver {

        public MotiveChangeReciever() {
            super();
        }

        @SuppressWarnings("unchecked")
        @Override
        public synchronized void workingMemoryChanged(WorkingMemoryChange _wmc) throws CASTException {


            String wmas = WMLogger.WMAToString(_wmc.address);
            WorkingMemoryAddress wma = _wmc.address;


            boolean success;
            switch (_wmc.operation) {
                case OVERWRITE:
                    getLogger().trace("overwrite");
                    success = trueplace.remove(wma) || placeholder.remove(wma);
                case ADD:
                    getLogger().trace("add");
                    Place p = getMemoryEntry(wma, Place.class);
                    success = add(p, wma);
                    break;
                case DELETE:
                    getLogger().trace("delete");
                    success = trueplace.remove(wma) || placeholder.remove(wma);
                    break;
                default:
                    success = true;
                    getLogger().warn("WMChange operation ignored in SetSizeLogger: " + _wmc.operation);
                    break;
            }

            if (!success) {
                getLogger().warn("Failure in PlaceSetSizeLogger");
            }

            // output the set size to logs
            XMLTag logTag = new XMLTag("MOTIVESETSIZE");
            logTag.addCastTimeAttr(_wmc.timestamp);
            logTag.addChild(setToXML(trueplace, "trueplace"));
            logTag.addChild(setToXML(placeholder, "placeholder"));
            String logString = logTag.toString();
            log(logString);
            println(logString);
        }

        private XMLTag setToXML(Set<WorkingMemoryAddress> set, String status) {
            XMLTag t = new XMLTag("SETSIZE");
            t.addAttr("type", PLACE_CLASS_NAME);
            t.addAttr("status", status);
            t.addContents(Integer.toString(set.size()), false);
            return t;
        }

        /**
         * Adds the Place's memory address to the correct set
         * @see Set#add(java.lang.Object) 
         */
        private boolean add(Place p, WorkingMemoryAddress wma) {
            switch (p.status) {
                case TRUEPLACE:
                    return trueplace.add(wma);
                case PLACEHOLDER:
                    return placeholder.add(wma);
                default:
                    // We are only interested in the above motives
                    getLogger().warn("Ignoring Place with status: " + p.status + " in PlaceSetSizeLogger");
                    return true; //
            }
        }
    }
}
