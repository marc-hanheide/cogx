/** _
 *  part of the CELM system.
 *  @author Dennis Stachowicz
 */

package celmarchitecture.global;


import java.util.Map;

import cast.architecture.SubarchitectureWorkingMemory;

/**
 * Provides a generic working memory class usable in conjunction with many C-ELM
 * processes. XarchChangeNotifications can be turned on and of in CAST config
 * files.
 */
public class DefaultWorkingMemory extends SubarchitectureWorkingMemory {

	public static final String configKeySendXarchCN = "--sendXarchCN";
	public static final String configKeyNoXarchCN = "--noXarchCN";

	public static final boolean sendXarchDefault = true;
	private boolean sendXarch = sendXarchDefault;

	private boolean localVerbose = false;
	private boolean verbose = GlobalSettings.verbose || localVerbose;

	public DefaultWorkingMemory() {
		super();
	}

	protected void configure(Map<String, String> config) {

		if (config.containsKey(configKeySendXarchCN))
			sendXarch = true;
		if (config.containsKey(configKeyNoXarchCN))
			sendXarch = false;

		if (verbose)
			println("setting sendXarchChangeNotifications to " + sendXarch);
		setSendXarchChangeNotifications(sendXarch);
	}

}
