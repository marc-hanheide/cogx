package binder.components;

import cast.architecture.ManagedComponent;

public class TrackingBinder extends ManagedComponent {
	
	public boolean LOGGING = true;
	
	
	public void start() {
		log("OK, dummy is working !");
	}
	
	private void log(String s) {
		if (LOGGING) {
		System.out.println("[Dummy] " + s);
		}
	}
}
