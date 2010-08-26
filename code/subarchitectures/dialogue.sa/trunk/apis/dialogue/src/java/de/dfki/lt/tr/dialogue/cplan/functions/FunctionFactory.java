package de.dfki.lt.tr.dialogue.cplan.functions;

import java.io.File;
import java.util.HashMap;
import java.util.List;

import de.dfki.lt.tr.dialogue.cplan.util.PluginLoader;


public class FunctionFactory {

  private static HashMap<String, Function> _registeredFunctions =
    new HashMap<String, Function>();

  public static Function get(String name) {
    return _registeredFunctions.get(name);
  }

  public static void register(Function function) {
    _registeredFunctions.put(function.name(), function);
  }

  public static void init(File pluginDirectory) {
    register(new DiscreteRandomFunction());
    register(new ConstantFunction("0"));
    register(new ConstantFunction("1"));
    if (pluginDirectory == null)
      return;
    if (! pluginDirectory.isDirectory()) {
      throw new IllegalArgumentException("Not a directory: " + pluginDirectory);
    }
    List<Function> pluginFunctions =
      new PluginLoader<Function>().loadPlugins(pluginDirectory);
    for (Function f : pluginFunctions) {
      register(f);
    }
  }

  /** Initialize only built-in functions */
  public static void init() {
    init(null);
  }
}
