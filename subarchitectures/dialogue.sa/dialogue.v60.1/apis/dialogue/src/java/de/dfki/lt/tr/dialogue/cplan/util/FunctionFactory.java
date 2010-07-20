package de.dfki.lt.tr.dialogue.cplan.util;

import java.util.HashMap;


public class FunctionFactory {

  private static HashMap<String, Function> _registeredFunctions;

  public static Function get(String name) {
    return _registeredFunctions.get(name);
  }
}
