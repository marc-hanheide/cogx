package castutils.experimentation;



import org.apache.log4j.Logger;

import cast.architecture.ManagedComponent;

public class LogFlooder extends ManagedComponent {

	/* (non-Javadoc)
	 * @see cast.core.CASTComponent#runComponent()
	 */
	@Override
	protected void runComponent() {
		Logger logger = getLogger();
		while(isRunning()) {
			getLogger().trace("TRACE flooder");
			getLogger().debug("DEBUG flooder");
			getLogger().info("INFO  flooder");
		}
	}

}
