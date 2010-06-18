package castutils.castextensions;

import java.io.FileInputStream;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.ObjectInputStream;
import java.io.ObjectOutputStream;
import java.util.concurrent.LinkedBlockingQueue;

import cast.CASTException;
import cast.architecture.ManagedComponent;
import cast.architecture.WorkingMemoryChangeReceiver;
import cast.cdl.WorkingMemoryChange;
import cast.cdl.WorkingMemoryOperation;

public class WMEntryQueue extends
		LinkedBlockingQueue<WMEntryQueue.WMEntryQueueElement> implements
		WorkingMemoryChangeReceiver {

	public class WMEntryQueueElement {
		/**
		 * @param addr
		 * @param entry
		 */
		WMEntryQueueElement(WorkingMemoryChange wmc, Ice.ObjectImpl entry) {
			this.wmc = wmc;
			this.entry = entry;
		}

		WorkingMemoryChange wmc;
		Ice.ObjectImpl entry;

		/**
		 * @return the address
		 */
		public WorkingMemoryChange getEvent() {
			return wmc;
		}

		/**
		 * @return the entry
		 */
		public Ice.ObjectImpl getEntry() {
			return entry;
		}
		
        @Override
		public String toString() {
            String name;
            if (entry != null) {
                    name = entry.getClass().getName();
            }
            else {
                    name = "unknown object";
            }

            switch (wmc.operation) {
            case ADD:
                    return String.format("add %s at address %s by %s",
                                    name, wmc.address.id, wmc.src);
            case OVERWRITE:
                    return String.format("overwrite %s at address %s by %s",
                                    name, wmc.address.id, wmc.src);
            case DELETE:
                    return String.format("delete object at address %s by %s",
                    				wmc.address.id, wmc.src);
            default:
                    assert false;
            }
            return "";
    }

		
	}

	/**
	 * @param component
	 *            the component that is used to access the memory
	 */
	public WMEntryQueue(ManagedComponent component) {
		super();
		this.component = component;
	}

	/**
	 * 
	 * @param component
	 * 			the component that is used to access the memory
	 * @param filename
	 */
	public WMEntryQueue(ManagedComponent component, String filename) {
		this(component);
        try
        {
                FileInputStream fis = new FileInputStream(filename);
                ObjectInputStream in = new ObjectInputStream(fis);
                int count = in.readInt();
                for (int i=0; i < count; i++) {
                		WorkingMemoryChange wmc = (WorkingMemoryChange) in.readObject();
                		Ice.ObjectImpl obj = (Ice.ObjectImpl) in.readObject();
                		this.put(new WMEntryQueueElement(wmc, obj));
                }
                in.close();
        }
        catch(IOException e) {
                e.printStackTrace();
        } catch(ClassNotFoundException e) {
                e.printStackTrace();
		} catch (InterruptedException e) {
			e.printStackTrace();
		}
		
	}

	/**
	 * 
	 */
	private static final long serialVersionUID = -7628018827365709394L;

	protected ManagedComponent component;

	@Override
	public synchronized void workingMemoryChanged(WorkingMemoryChange wmc)
			throws CASTException {
		try {
			if (wmc.operation != WorkingMemoryOperation.DELETE) {
				Ice.ObjectImpl o = component.getMemoryEntry(wmc.address,
						Ice.ObjectImpl.class);
				WMEntryQueueElement qe = new WMEntryQueueElement(wmc, o);
				this.put(qe);

			} else {
				WMEntryQueueElement qe = new WMEntryQueueElement(wmc, null);
				this.put(qe);
			}
		} catch (InterruptedException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
	}
	
	public void save(String filename) {
		try {
            FileOutputStream fos = new FileOutputStream(filename);
            ObjectOutputStream out = new ObjectOutputStream(fos);
            out.writeInt(size());
            for (WMEntryQueueElement e : this) {
                    out.writeObject(e.wmc);
                    out.writeObject(e.entry);
            }
            out.close();
		}
		catch (IOException e) {
			e.printStackTrace();
		}
		
	}

}
