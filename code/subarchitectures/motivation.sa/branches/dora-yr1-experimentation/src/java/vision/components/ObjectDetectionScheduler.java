/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
package vision.components;

import NavData.ObjObs;
import SpatialData.Place;
import SpatialData.PlaceStatus;
import cast.CASTException;
import cast.architecture.ChangeFilterFactory;
import cast.architecture.ManagedComponent;
import cast.architecture.WorkingMemoryChangeReceiver;
import cast.cdl.WorkingMemoryChange;
import cast.cdl.WorkingMemoryOperation;
import cast.core.CASTUtils;
import java.util.ArrayList;
import java.util.Collections;
import java.util.HashSet;
import java.util.Iterator;
import java.util.Map;
import java.util.Set;

/**
 * syntax for adding an event
 * "--object-detection &lt;label1&gt;:&lt;PlaceNumber1&gt;,&lt;PlaceNumber2&gt;#
 * &lt;label2&gt;:&lt;PlaceNumber1&gt;,&lt;PlaceNumber2&gt;" <br/>
 *  eg. --object-detection "poster:5#cup:1,7,8"
 * @author ug85jxh
 */
public class ObjectDetectionScheduler extends ManagedComponent implements
        WorkingMemoryChangeReceiver {

    public static final String OPTION_PREFIX = "--object-detection";
    public static final String TYPE_SEPARATOR = "#";
    public static final String TIME_SEPARATOR = ",";
    private static long IDS = 1000000; // start at a high number to avoid conflicts with ObjectSearch.cpp
    /**
     * list of VisualObjects that should be posted when so many FNodes exist
     */
    private final ArrayList<ObjectDetectionEvent> objectDetectionSchedule;
    /**
     * the set of TRUEPLACES that currently exist
     */
    private final Set<Long> places;

    public ObjectDetectionScheduler() {
        this.places = new HashSet<Long>();
        this.objectDetectionSchedule = new ArrayList<ObjectDetectionEvent>();

    }

    @Override
    protected void start() {
        // register an ice server so we can add objects on command/remotely
//        registerIceServer(null, null);

        addChangeFilter(ChangeFilterFactory.createTypeFilter(
                Place.class, WorkingMemoryOperation.ADD), this);

        addChangeFilter(ChangeFilterFactory.createTypeFilter(
                Place.class, WorkingMemoryOperation.DELETE), this);

        addChangeFilter(ChangeFilterFactory.createTypeFilter(
                Place.class, WorkingMemoryOperation.OVERWRITE), this);
    }

    @Override
    public void workingMemoryChanged(WorkingMemoryChange wmc) throws CASTException {

        if (objectDetectionSchedule.isEmpty()) {
            return; // no more events to post
        }

        synchronized (this) {

            Place place = getMemoryEntry(wmc.address, Place.class); // may throw ClassCastException

            if (!countPlace(place, wmc.operation)) {
                return;
            }

            Iterator<ObjectDetectionEvent> iter = objectDetectionSchedule.iterator();
            ObjectDetectionEvent event;
            while (iter.hasNext()) {
                event = iter.next();
                // if we need to observe more places for this event then end loop
                if (event.numPlaces > places.size()) {
                    break; // should be sorted,so break is more appropriate than continue
                }

                ObjObs obs = new ObjObs(
                        CASTUtils.getTimeServer().getCASTTime(),
                        event.label,
                        new long[]{IDS++},
                        new double[0],
                        new double[0]);

                println("adding " + event + " and its observation to working memory");
                addToWorkingMemory(newDataID(), obs);
                iter.remove(); // remove this visual object as it has now been used
            }
        }
    }

    @Override
    protected void configure(Map<String, String> config) {
        String options = config.get(OPTION_PREFIX);

        if (options == null) {
            return;
        }

        for (String option : options.split(TYPE_SEPARATOR)) {
            int colonPos = option.indexOf(":");
            String label = option.substring(0, colonPos).trim();
            String times = option.substring(colonPos + 1);
            for (String timeString : times.split(TIME_SEPARATOR)) {
                // add one to time as places are indexed from zero not 1
                int time = Integer.parseInt(timeString.trim()) + 1;
                objectDetectionSchedule.add(new ObjectDetectionEvent(time, label));
            }
        }

        // sort events
        Collections.sort(objectDetectionSchedule);
        println("schedule -- " + objectDetectionSchedule.toString());

    }

    /**
     * Do we care about this place? Returns true if we do, false otherwise
     * @param status the status of the place
     * @param operation the operation that was performed on the place
     * @return whether this place has effect the places field or not
     */
    private boolean countPlace(Place place, WorkingMemoryOperation operation) {

        switch (operation) {
            case ADD: // another place exists

                if (place.status == PlaceStatus.TRUEPLACE) {
                    // only add if the place is a true place
                    return places.add(place.id);
                } else {
                    return false;
                }

            case DELETE: // one less place exists
                // returns true if and only if we already knew about this place
                return places.remove(place.id);

            case OVERWRITE: // place has changed status
                if (place.status == PlaceStatus.TRUEPLACE) {
                    // if true place then add
                    // if we already know this place to be a true place then this will return false
                    return places.add(place.id);
                } else {
                    // if not true place then remove
                    // if we were not aware of this place then this will return false
                    return places.remove(place.id);
                }

            default: // no change to numer of places -- may as well return now
                return false;
        }

    }

    class ObjectDetectionEvent implements Comparable<ObjectDetectionEvent> {

        /**
         *  at what number of places this event should occur
         */
        public final int numPlaces;
        /**
         * the label of the object that will be detected
         */
        public final String label;

        public ObjectDetectionEvent(int numPlaces, String label) {
            this.numPlaces = numPlaces;
            this.label = label;
        }

        @Override
        public int compareTo(ObjectDetectionEvent o) {
            return numPlaces - o.numPlaces;
        }

        @Override
        public String toString() {
            return label + ':' + numPlaces;
        }
    }
}
