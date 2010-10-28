/**
 * 
 */
package castutils.viewer;

import java.util.Map;
import java.util.StringTokenizer;

import castutils.castextensions.WMEntrySet;
import Ice.ObjectImpl;
import cast.architecture.ManagedComponent;

/**
 * @author marc
 * 
 */
public class ViewerCastComponent extends ManagedComponent {

	ViewerGUI gui;
	WMEntrySet entrySet;

	public ViewerCastComponent() {
		super();
		entrySet = WMEntrySet.create(this);
		gui = new ViewerGUI();
		gui.pack();
		gui.setVisible(true);
		
		entrySet.setHandler(gui);
		gui.setSize(1000, 400);
		
	}

	/*
	 * (non-Javadoc)
	 * 
	 * @see cast.core.CASTComponent#start()
	 */
	@Override
	protected void start() {
		entrySet.start();
		super.start();
	}

	@SuppressWarnings("unchecked")
	@Override
	protected void configure(Map<String, String> arg0) {
		super.configure(arg0);
		String subscrStr = arg0.get("--subscribe");
		if (subscrStr != null) {
			StringTokenizer st = new StringTokenizer(subscrStr, ",");
			while (st.hasMoreTokens()) {
				String className = st.nextToken();
				className=className.trim();
				try {
					System.out.println("add type '" + className+"'");
					ClassLoader.getSystemClassLoader().loadClass(className);
					entrySet.addType((Class<? extends ObjectImpl>) Class
							.forName(className));
				} catch (ClassNotFoundException e) {
					println("trying to register for a class that doesn't exist.");
					e.printStackTrace();
				}
			}
		}
	}

	/* (non-Javadoc)
	 * @see cast.core.CASTComponent#stop()
	 */
	@Override
	protected void stop() {
		gui.setVisible(false);
		gui.dispose();
	
	}

}
