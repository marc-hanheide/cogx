/**
 * @author hanheidm
 */
package kinect.components;

import java.util.Map;
import java.util.Map.Entry;

import kinect.slice.KinectPerson;
import kinect.slice.PersonDetectorInterface;
import kinect.slice.PersonDetectorInterfacePrx;
import cast.CASTException;
import cast.architecture.ManagedComponent;

/**
 * @author hanheidm
 * 
 */
public class PersonDetectorRunner extends ManagedComponent {

	private static final String PCSERVER_DEFAULT = "pcserver";
	private static final String SERVER_KEY = "--server";
	private String serverId = PCSERVER_DEFAULT;
	private PersonDetectorInterfacePrx server = null;

	@Override
	protected void configure(Map<String, String> config) {
		if (config.containsKey(SERVER_KEY)) {
			serverId = config.get(SERVER_KEY);
		}
	}

	@Override
	protected void runComponent() {
		while (isRunning()) {
			if (server == null)
				break;
			Map<Integer, KinectPerson> persons = server.getPersons();
			for (Entry<Integer, KinectPerson> p : persons.entrySet()) {
				println("person found: id=" + p.getKey() + " size="
						+ p.getValue().size);
			}
			sleepComponent(1000);
		}
	}

	@Override
	protected void start() {
		try {
			server = getIceServer(serverId, PersonDetectorInterface.class,
					PersonDetectorInterfacePrx.class);
		} catch (CASTException e) {
			logException(e);
		}
	}

}
