package cast.server;

import Ice.Current;
import Ice.LocalObjectHolder;
import Ice.Object;
import Ice.ServantLocator;
import Ice.UserException;
import cast.core.CASTUtils;

public class ComponentLocator implements ServantLocator {

	public ComponentLocator() {
	}

	public void deactivate(String _arg0) {

	}

	public void finished(Current _arg0, Object _arg1, java.lang.Object _arg2)
			throws UserException {

	}
	
	public int ice_hash() {
		return hashCode();
	}

	public Object locate(Current _current, LocalObjectHolder _arg1) {
		synchronized (this) {

			String name = _current.id.name;
			String cat = _current.id.category;

//			System.out.println("locate: " + name + " " + cat);

			Object servant = _current.adapter.find(_current.id);
			if (servant == null) {
				try {
					servant = CASTUtils.createServant(_current.id, _current.adapter);
					if (servant != null) {
						_current.adapter.add(servant, _current.id);
						}
				} catch (Exception e) {
					throw new RuntimeException("Error creating component "
							+ name + " of class " + cat + ". Exception error: "
							+ e.getLocalizedMessage(), e);
				}
			}

			assert (servant != null);

			return servant;
		}
	}


}
