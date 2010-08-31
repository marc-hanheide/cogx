/** _
 *  part of the ELM system.
 *  @author Dennis Stachowicz
 */

package elm.util;

import java.util.Vector;
import java.util.concurrent.locks.Condition;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;


public class CircularBuffer<E> {


    private final Lock lock = new ReentrantLock();
    private final Condition notFull  = lock.newCondition(); 
    private final Condition notEmpty = lock.newCondition();
 
    private final Object[] buffer;
    private final int capacity;

    private int first = 0;
    private int last  = -1;
    private int size  = 0;

    public CircularBuffer(int capacity) throws ArrayIndexOutOfBoundsException {
			  
	if (capacity < 1)
	    throw new ArrayIndexOutOfBoundsException("buffer size needs to be at least 1!");
	
	this.capacity = capacity;
	buffer = new Object[capacity];
    }


    /**
     *  blocking operation as in java.util.concurrent.BlockingQueue
     *  @see java.util.concurrent.BlockingQueue
     */
    public void put(E e) throws InterruptedException {

	lock.lock();

	try {
	    while (size == capacity)
		notFull.await();
	    
	    last = (last + 1) % capacity;
	    size++;
		
	    buffer[last] = e;	    
	    elementInserted(last);

	    notEmpty.signal();

	} finally {
	    lock.unlock();
	}
    }    

    /**
     *  blocking operation as in java.util.concurrent.BlockingQueue
     *  @see java.util.concurrent.BlockingQueue
     */
    @SuppressWarnings("unchecked") 
    public E take() throws InterruptedException {

	E e = null;
	lock.lock();

	try {

	    while (size == 0)
		notEmpty.await();

	    elementDeleted(first);
	    e = (E) buffer[first];
	    first = (first + 1) % capacity;
	    size--;	    

	    notFull.signal();
	    return e;

	} finally {
	    lock.unlock();
	}
    }



    public int add(E e) throws CircularBufferException {

	lock.lock();

	try {

	    if (full())
		throw new CircularBufferException("CircularBuffer is full.");

	    last = (last + 1) % capacity;
	    size++;
	    
	    buffer[last] = e;
	    elementInserted(last);

	    notEmpty.signal();
	    return last;

	} finally {
	    lock.unlock();
	}
    }

    public int addOrOverwrite(E e) {

	lock.lock();

	try {

	    if (size < capacity)
		size++;
	    else {
		elementDeleted(first);
		first++;
	    }

	    last = (last + 1) % capacity;
	
	    buffer[last] = e;
	    elementInserted(last);

	    notEmpty.signal();
	    return last;

	} finally {
	    lock.unlock();
	}
    }

    private int internalIndex(int index) {

	return (first + index) % capacity;
    }

    public int size() {

	return size;
    }
    
    public int capacity() {
       
	return capacity;
    }

    public boolean full() {

	if (size >= capacity)
	    return true;
	return false;
    }

    public boolean empty() {

	return (size == 0);
    }

    @SuppressWarnings("unchecked") 
    public E get(int index)
	throws ArrayIndexOutOfBoundsException {

	lock.lock();

	try {	

	    if (index < 0 || index >= size)
		throw new ArrayIndexOutOfBoundsException("index too large or negative: " + index);
	    if (size < 1)
		throw new ArrayIndexOutOfBoundsException("nothing in this CircularBuffer!");
	    return (E) buffer[internalIndex(index)];

	} finally {
	    lock.unlock();
	}
    }

    @SuppressWarnings("unchecked") 
    protected E getAtInternalIndex(int index)
	throws ArrayIndexOutOfBoundsException {

	lock.lock();

	try {	

	    if (index < 0 || index >= size)
		throw new ArrayIndexOutOfBoundsException("index too large or negative: " + index);
	    if (size < 1)
		throw new ArrayIndexOutOfBoundsException("nothing in this CircularBuffer!");
	    return (E) buffer[index];

	} finally {
	    lock.unlock();
	}
    }


    /**
     *  returns the events currently having indices startIndex to endIndex (inclusive)
     */
    public Vector<E> getRange(int startIndex, int endIndex) 
	throws ArrayIndexOutOfBoundsException {

	lock.lock();

	try {	

	    if (startIndex > endIndex)
		return null;

	    if (startIndex < 0)
		throw new ArrayIndexOutOfBoundsException("index negative: " + startIndex);

	    if (endIndex >= size())
		throw new ArrayIndexOutOfBoundsException("index too large: " + endIndex);

	    if (size() < 1)
		throw new ArrayIndexOutOfBoundsException("nothing in this CircularBuffer!");

	    int length = endIndex - startIndex + 1;
	    Vector<E> v = new Vector<E>(length);
	    for (int i = 0; i < length; i++)
		v.add(getAtInternalIndex(startIndex + i));
	   	    
	    return v;

	} finally {
	    lock.unlock();
	}
    } 

    
    public void discard(int numberOfObjects) 
	throws CircularBufferException {

	lock.lock();

	try {
	    if (numberOfObjects == 0)
		return;	
	    
	    if (numberOfObjects < 0)
		throw new CircularBufferException("number of objects to delete must " + 
						  "not be negative");
	
	    if (numberOfObjects > size())
		throw new CircularBufferException("trying to discard " + 
						  numberOfObjects + 
						  " objects while only " + 
						  size() + " objects are available");

	    for (int i = 0; i < numberOfObjects; i++)
		elementDeleted(internalIndex(i));

	    first = (first + numberOfObjects) % capacity;
	    size -= numberOfObjects;

	    notFull.signal();

	} finally {
	    lock.unlock();
	}
    }


    public void flush() 
	throws CircularBufferException {

	lock.lock();

	try {
	    discard(size());
	} finally {
	    lock.unlock();
	}
    }


    /**
     *  Method which can be overridden by a derived class to get notifications
     *  about deleted elements.
     */
    protected void elementDeleted(int internalIndex) {
    }

    /**
     *  Method which can be overridden by a derived class to get notifications
     *  about inserted elements.
     */
    protected void elementInserted(int internalIndex) {
    }

}



