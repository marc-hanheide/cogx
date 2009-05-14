package coma.aux;

public class InstanceUnknownException extends Exception {

	/**
	 * 
	 */
	private static final long serialVersionUID = 1363648951000261632L;

	public InstanceUnknownException() {
		// TODO Auto-generated constructor stub
	}

	public InstanceUnknownException(String _ins) {
		super("Unknown instance: " + _ins);
		// TODO Auto-generated constructor stub
	}

}