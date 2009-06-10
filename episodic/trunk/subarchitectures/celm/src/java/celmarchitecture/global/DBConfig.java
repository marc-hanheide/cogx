/** _
 *  part of the CELM system.
 *  @author Dennis Stachowicz
 */

package celmarchitecture.global;

import java.util.Properties;

/**
 *  Unifies the configuration of ElmWriter and Recollector's 
 *  database parameters.
 */
public class DBConfig {
    
    public static final String configKeyServer  = "--server";
    public static final String configKeyDB      = "--db";
    public static final String configKeyUser    = "--user";
    public static final String configKeyPasswd  = "--passwd";
    
    public String server                        = "localhost";
    public String name                          = "elm";
    public String user                          = "elm";
    public String passwd                        = "somepw";
    
    
    public void configure(Properties config) {
	
	if (config.containsKey(configKeyServer)) 
	    server = config.getProperty(configKeyServer);
	if (config.containsKey(configKeyDB)) 
	    name   = config.getProperty(configKeyDB);
	if (config.containsKey(configKeyUser)) 
	    user   = config.getProperty(configKeyUser);
	if (config.containsKey(configKeyPasswd)) 
	    passwd = config.getProperty(configKeyPasswd);

    }
    
    
}