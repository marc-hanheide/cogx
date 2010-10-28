package castutils.viewer.plugins;

import java.util.Vector;

public interface Plugin {
	public Vector<Object> toVector(Ice.ObjectImpl iceObject);
}
