/**
 * 
 */
package motivation.util.castextensions;

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
import motivation.slice.Motive;

/**
 * @author marc
 * 
 */
public class MotiveSetSizeLogger extends ManagedComponent {

    static final String MOTIVE_CLASS_NAME = "motivation.slice.Motive";

    @Override
    public void start() /* throws UnknownSubarchitectureException */ {
        // create log
        getLogger().info("MotiveSetSizeLogger Started");
        // register listener
        register();
    }

    public <T extends Ice.ObjectImpl> void register() {
        try {
            Class<? extends ObjectImpl> type = (Class<? extends ObjectImpl>) Class.forName(MOTIVE_CLASS_NAME);
            getLogger().info("register listener");
            MotiveChangeReciever sscr = new MotiveChangeReciever();
            addChangeFilter(ChangeFilterFactory.createTypeFilter(type, WorkingMemoryOperation.ADD), sscr);
            addChangeFilter(ChangeFilterFactory.createTypeFilter(type, WorkingMemoryOperation.DELETE), sscr);
            addChangeFilter(ChangeFilterFactory.createTypeFilter(type, WorkingMemoryOperation.OVERWRITE), sscr);
        } catch (ClassNotFoundException ex) {
            getLogger().fatal("couldn't load motive class in MotiveSetSizeLogger.register", ex);
        }
    }
    Set<WorkingMemoryAddress> active = new HashSet();
    Set<WorkingMemoryAddress> surfaced = new HashSet();
    Set<WorkingMemoryAddress> unsurfaced = new HashSet();

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
                    success = active.remove(wma) || surfaced.remove(wma) || unsurfaced.remove(wma);
                case ADD:
                    Motive m = getMemoryEntry(wma, Motive.class);
                    success = add(m, wma);
                    break;
                case DELETE:
                    success = (active.remove(wma) || surfaced.remove(wma) || unsurfaced.remove(wma));
                    break;
                default:
                    success = false;
                    getLogger().warn("WMChange operation ignored in SetSizeLogger: " + _wmc.operation);
                    break;
            }
            if(!success) {
                getLogger().warn("error in MotiveSetSizeLogger");
            }

            // output the set size to logs
            XMLTag logTag = new XMLTag("MOTIVESETSIZE");
            logTag.addCastTimeAttr(_wmc.timestamp);
            logTag.addChild(setToXML(active, "active"));
            logTag.addChild(setToXML(surfaced, "surfaced"));
            logTag.addChild(setToXML(unsurfaced, "unsurfaced"));
            String logString = logTag.toString();
            log(logString);
            println(logString);
        }

        private XMLTag setToXML(Set<WorkingMemoryAddress> set, String status) {
            XMLTag t = new XMLTag("SETSIZE");
            t.addAttr("type", MOTIVE_CLASS_NAME);
            t.addAttr("status",status);
            t.addContents(Integer.toString(set.size()),false);
            return t;
        }

        /**
         * adds address of the motive to the correct list
         */
        private boolean add(Motive m, WorkingMemoryAddress wma) {
            switch (m.status) {
                case ACTIVE:
                    return active.add(wma);
                case SURFACED:
                    return surfaced.add(wma);
                case UNSURFACED:
                    return unsurfaced.add(wma);
                default:
                    // We are only interested in the above motives
                    getLogger().debug("Ignoring motive with status: " + m.status + " in MotiveSetSizeLogger");
                    return true;
            }
        }
    }
}