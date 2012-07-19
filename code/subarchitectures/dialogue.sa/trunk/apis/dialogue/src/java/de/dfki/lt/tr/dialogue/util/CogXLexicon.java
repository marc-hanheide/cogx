package de.dfki.lt.tr.dialogue.util;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

public class CogXLexicon {

    public static boolean isObjectType(String s) {
        final List<String> objectTypes = new ArrayList<String>();
//        objectTypes.add("box");
        objectTypes.add("ball");
        objectTypes.add("mug");
        objectTypes.add("block");
        objectTypes.add("cerealbox");
        objectTypes.add("teabox");
        objectTypes.add("drinkcan");
        objectTypes.add("noodle");
        objectTypes.add("milk");
        objectTypes.add("bottle");
        
        return objectTypes.contains(s);
    }
    
    public static String simplifyObjectTypes(String s) {
        final Map<String, String> simplifications = new HashMap<String, String>();
        simplifications.put("tea box", "teabox");
        simplifications.put("cereal box", "cerealbox");
        simplifications.put("box of cereals", "cerealbox");
        simplifications.put("drink can", "drinkcan");
        simplifications.put("milk box", "milk");

        String result = s;
        for (Map.Entry<String, String> exp : simplifications.entrySet()) {
            result = result.replaceFirst(exp.getKey(), exp.getValue());
        }
        return result;
    }
    
    public static String beautifyObjectTypes(String s) {
        final Map<String, String> beautifications = new HashMap<String, String>();
        beautifications.put("teabox", "tea box");
        beautifications.put("cerealbox", "cereal box");
        beautifications.put("drinkcan", "drink can");
        
        String result = s;
        for (Map.Entry<String, String> exp : beautifications.entrySet()) {
            result = result.replaceFirst(exp.getKey(), exp.getValue());
        }
        return result;
    }

    
}
