/**
 * 
 */
package spatial.manual;

import java.util.Map;

import cast.architecture.ManagedComponent;

/**
 * @author cogx
 *
 */
public class ManualNavControl extends ManagedComponent {

	@Override
	protected void configure(Map<String, String> config) {
		// TODO Auto-generated method stub
		super.configure(config);
	}

	@Override
	protected void runComponent() {
		new ManualNavGUI(this);
	}

	@Override
	protected void start() {
		
	}
	
	
	

}
