/*

 */
package de.dfki.lt.tr.dialogue.cplan.util;

/**
 * @author Bernd Kiefer, inspired by code from Andrea Sindico and AUCOM S.R.L.
 */

import java.io.File;
import java.io.IOException;
import java.net.MalformedURLException;
import java.net.URL;
import java.net.URLClassLoader;
import java.util.Enumeration;
import java.util.LinkedList;
import java.util.List;
import java.util.jar.JarEntry;
import java.util.jar.JarFile;

import org.apache.log4j.Logger;

public class PluginLoader<PluginType>{

  private static Logger logger = Logger.getLogger("UtterancePlanner");

  private URLClassLoader classloader = null;

  /**
   *it takes as parameter an id representing a pluginEntryId and returns an
   * instance of that PluginEntry
   *
   */
  @SuppressWarnings("unchecked")
  private PluginType getPlugin(String name) {
    name = name.replace(File.separatorChar, '.');
    name = name.substring(0, name.length() - 6); // remove ".class"
    try {
      Object o = classloader.loadClass(name).newInstance();
      return (PluginType) o;
    } catch (ClassCastException e) {
      // we indicate a non-matching entry by returning null
    } catch (Exception e) {
      logger.warn(e.getMessage());
     }
    return null;
  }


  /** Load all plugins in the given JAR
   */
  private void loadAllPlugins(File file, List<PluginType> result) {
    JarFile jarFile = null;
    try {
      jarFile = new JarFile(file);
      for (Enumeration<JarEntry> e = jarFile.entries(); e.hasMoreElements();) {
        JarEntry jarEntry = e.nextElement();
        String className = jarEntry.getName();
        if (className.endsWith((".class"))) {
          PluginType plugin = getPlugin(className);
          if (plugin != null) {
            logger.info("Added " + plugin.getClass().getSimpleName());
            result.add(plugin);
          }
        }
      }
    } catch (IOException e) {
      logger.warn(e.getMessage());
    } finally {
      if (jarFile != null) {
        try {
          jarFile.close();
        } catch (IOException ioe) {
        }
      }
    }
  }

  /** Iterate over all .jar files in pluginsPath and create plug-in objects for
   *  all subclasses of PluginType
   */
  public List<PluginType> loadPlugins(File pluginsPath) {
    List<PluginType> result = new LinkedList<PluginType>();
    String[] jar =
      pluginsPath.list(new java.io.FilenameFilter() {
        public boolean accept(File dir, String filename) {
          return filename.endsWith(".jar");
        }
      });

    if (jar.length == 0)
      return result;

    URL[] url = new URL[jar.length];
    for (int i = 0; i < jar.length; i++) {
      File jarFile = new File(pluginsPath, jar[i]);
      try {
        url[i] = jarFile.toURI().toURL();
      } catch (MalformedURLException ex) {
        logger.warn(ex.getMessage());
      }
    }
    classloader = new URLClassLoader(url);
    for (int i = 0; i < jar.length; i++) {
      logger.info("Loading " + jar[i]);
      loadAllPlugins(new File(pluginsPath, jar[i]), result);
    }
    return result;
  }
}
