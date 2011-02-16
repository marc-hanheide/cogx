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

public class WMEntryQueue<T extends Ice.Object> extends
		LinkedBlockingQueue<WMEntryQueue.WMEntryQueueElement<T>> implements
		WorkingMemoryChangeReceiver {

	public static class WMEntryQueueElement<T2 extends Ice.Object> {
		/**
		 * @param addr
		 * @param entry
		 */
		WMEntryQueueElement(WorkingMemoryChange wmc, T2 entry) {
			this.wmc = wmc;
			this.entry = entry;
		}

		WorkingMemoryChange wmc;
		T2 entry;

		/**
		 * @return the address
		 */
		public WorkingMemoryChange getEvent() {
			return wmc;
		}

		/**
		 * @return the entry
		 */
		public T2 getEntry() {
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

	private Class<T> type;


	/**
	 * @param component
	 *            the component that is used to access the memory
	 */
	public WMEntryQueue(ManagedComponent component, Class<T> type) {
		super();
		this.type=type;
		this.component = component;
	}

	/**
	 * 
	 * @param component
	 * 			the component that is used to access the memory
	 * @param filename
	 */
	public WMEntryQueue(ManagedComponent component, String filename, Class<T> type) {
		this(component, type);
        try
        {
                FileInputStream fis = new FileInputStream(filename);
                ObjectInputStream in = new ObjectInputStream(fis);
                int count = in.readInt();
                for (int i=0; i < count; i++) {
                		WorkingMemoryChange wmc = (WorkingMemoryChange) in.readObject();
                		T obj = type.cast(in.readObject());
                		this.put(new WMEntryQueueElement<T>(wmc, obj));
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
				T o = component.getMemoryEntry(wmc.address,
						type);
				WMEntryQueueElement<T> qe = new WMEntryQueueElement<T>(wmc, o);
				this.put(qe);

			} else {
				WMEntryQueueElement<T> qe = new WMEntryQueueElement<T>(wmc, null);
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
            for (WMEntryQueueElement<T> e : this) {
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
