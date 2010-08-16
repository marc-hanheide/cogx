/**
 * 
 */
package motivation.util.castextensions;

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
            Class<? extends ObjectImpl> type = (Class<? extends ObjectImpl>) Class.forName(PLACE_CLASS_NAME);
            getLogger().info("register listener");
            MotiveChangeReciever sscr = new MotiveChangeReciever();
            addChangeFilter(ChangeFilterFactory.createTypeFilter(type, WorkingMemoryOperation.ADD), sscr);
            addChangeFilter(ChangeFilterFactory.createTypeFilter(type, WorkingMemoryOperation.DELETE), sscr);
            addChangeFilter(ChangeFilterFactory.createTypeFilter(type, WorkingMemoryOperation.OVERWRITE), sscr);
        } catch (ClassNotFoundException ex) {
            getLogger().fatal("couldn't load motive class in MotiveSetSizeLogger.register", ex);
        }
    }
    Set<WorkingMemoryAddress> trueplace = new HashSet();
    Set<WorkingMemoryAddress> placeholder = new HashSet();

    public class MotiveChangeReciever implements WorkingMemoryChangeReceiver {

        /** Has to be a Set&lt;String&gt; as WorkingMemoryAddress doesn't 
         * correctly implement .equals or .hashCode() */
//        Set<String> addresses = new HashSet<String>();
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
                    success = trueplace.remove(wma) || placeholder.remove(wma);
                case ADD:
                    Place m = getMemoryEntry(wma, Place.class);
                    Set s = getSetFor(m);

                    if (m == null) {
                        return;
                    } else {
                        s.add(m);
                    }
                    break;
                case DELETE:
                    success = trueplace.remove(wma) || placeholder.remove(wma);
                    break;
                default:
                    getLogger().warn("WMChange operation ignored in SetSizeLogger: " + _wmc.operation);
                    break;
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
            t.addAttr("status",status);
            t.addContents(Integer.toString(set.size()),false);
            return t;
        }

        /**
         * returns the set that corresponds to the motive type,
         * or null if we are not interested in a motive with this status
         * @returns a motive set or null
         */
        private Set<WorkingMemoryAddress> getSetFor(Place p) {
            switch (p.status) {
                case TRUEPLACE:
                    return trueplace;
                case PLACEHOLDER:
                    return placeholder;
                default:
                    // We are only interested in the above motives
                    getLogger().debug("Ignoring Place with status: " + p.status + " in PlaceSetSizeLogger");
                    return null;
            }
        }
    }
}
