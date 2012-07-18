package de.dfki.lt.tr.dialogue.interpret;

import cast.cdl.WorkingMemoryAddress;
import de.dfki.lt.tr.beliefs.slice.intentions.InterpretedIntention;
import de.dfki.lt.tr.beliefs.slice.intentions.PossibleInterpretedIntentions;
import java.util.*;
import org.apache.log4j.Logger;

public class PossibleReadingsFilter {

    private final Logger logger;
    
    public PossibleReadingsFilter(Logger logger) {
        this.logger = logger;
    }

    public PossibleInterpretedIntentions filterForAboutness(PossibleInterpretedIntentions pii, InterpretedIntention best) {

        Map<WorkingMemoryAddress, InterpretedIntention> firstPass = new HashMap<WorkingMemoryAddress, InterpretedIntention>();
        
        logger.debug("about to filter out intentions that are structurally different from the best one");
        
        // filter out those that are structurally different
        for (Map.Entry<WorkingMemoryAddress, InterpretedIntention> entry : pii.intentions.entrySet()) {
            
            WorkingMemoryAddress wma = entry.getKey();
            InterpretedIntention iint = entry.getValue();
            
            // the rest only if not equivalent with the best one
            if (equivalentApartFromAboutness(best, entry.getValue())) {
                logger.trace("including [" + wma.id + "," + wma.subarchitecture + "] in the first pass: equivalent");
                firstPass.put(wma, iint);
            }
            else {
                logger.trace("EXcluding [" + wma.id + "," + wma.subarchitecture + "] in the first pass: not equivalent");
            }
        }

        logger.debug("about to filter out duplicates in the about field");
        
        // TODO: filter out duplicates
        Map<WorkingMemoryAddress, InterpretedIntention> secondPass = new HashMap<WorkingMemoryAddress, InterpretedIntention>();
        if (best.addressContent.containsKey("about")) {
            List<WorkingMemoryAddress> abouts = getAbouts(firstPass);
            
            for (WorkingMemoryAddress about : abouts) {
                Map.Entry<WorkingMemoryAddress, InterpretedIntention> bestEntry = bestForAbout(firstPass, about);
                if (bestEntry != null) {
                    secondPass.put(bestEntry.getKey(), bestEntry.getValue());
                }
            }
        }
        else {
            logger.trace("not applicable, taking them all");
            secondPass.putAll(firstPass);
        }
        
        pii.intentions = secondPass;
        return pii;
    }
    
    public static boolean equivalentApartFromAboutness(InterpretedIntention mask, InterpretedIntention toCheck) {
        return subsumesOneWay(null, mask.stringContent, toCheck.stringContent)
                && subsumesOneWay(null, toCheck.stringContent, mask.stringContent)
                && subsumesOneWay("about", mask.addressContent, toCheck.addressContent)
                && subsumesOneWay("about", toCheck.addressContent, mask.addressContent);
    }
    
    public static <K, V> boolean subsumesOneWay(K ignore, Map<K, V> left, Map<K, V> right) {
        for (Map.Entry<K, V> e : left.entrySet()) {
            if (!e.getKey().equals(ignore)) {
                V v = right.get(e.getKey());
                if (!e.getValue().equals(v)) return false;
            }
        }
        return true;
    }

    public static List<WorkingMemoryAddress> getAbouts(Map<WorkingMemoryAddress, InterpretedIntention> iints) {
        List<WorkingMemoryAddress> result = new ArrayList<WorkingMemoryAddress>();
        
        for (InterpretedIntention iint : iints.values()) {
            WorkingMemoryAddress about = iint.addressContent.get("about");
            if (about != null && !result.contains(about)) {
                result.add(about);
            }
        }
        
        return result;
    }
    
    public static Map.Entry<WorkingMemoryAddress, InterpretedIntention> bestForAbout(Map<WorkingMemoryAddress, InterpretedIntention> iints, WorkingMemoryAddress about) {
        Map.Entry<WorkingMemoryAddress, InterpretedIntention> best = null;
        for (Map.Entry<WorkingMemoryAddress, InterpretedIntention> e : iints.entrySet()) {
            if (about.equals(e.getValue().addressContent.get("about"))) {
                if (best == null || best.getValue().confidence < e.getValue().confidence) {
                    best = e;
                }
            }
        }
        return best;
    }
    
    
}
