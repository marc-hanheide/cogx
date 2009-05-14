/**
 * 
 */
package cast.architecture.subarchitecture;

import cast.architecture.abstr.LocalWorkingMemoryAttachedComponent;
import cast.architecture.abstr.WorkingMemoryAttachedComponent;
import cast.architecture.abstr.WorkingMemoryReaderWriterProcess;
import cast.cdl.WorkingMemoryAddress;

/**
 * An exception that occurs if there is a problem with working memory read/write
 * consistency.
 * 
 * The working memory consistency model in CAST is based the "invalidate on
 * write" consistency model from the distributed shared memory literature.
 * Consistency is only relevant when overwriting a working memory entry,
 * additions and deletions are not effected by consistency. To make a consistent
 * overwrite, the writing component must have read the most recent version of
 * the working memory before overwriting it. This guarantees that no changes
 * will be lost during overwrites. If you attempt to overwrite a working memory
 * entry without having read the most recent version (or having read it at all)
 * you will get a {@link ConsistencyException}. Consistency checks are
 * performed in {@link LocalWorkingMemoryAttachedComponent} and
 * {@link WorkingMemoryAttachedComponent} and are triggered by
 * {@link WorkingMemoryReaderWriterProcess} and {@link PrivilegedManagedProcess}
 * overwriteWorkingMemory calls. Consistency state can be checked with
 * haveLatestVersion in {@link WorkingMemoryAttachedComponent}.
 * 
 * 
 * @author nah
 * 
 */
public class ConsistencyException extends WMException {

	/**
	 * 
	 */
	private static final long serialVersionUID = -5905620957952971402L;

	public ConsistencyException(WorkingMemoryAddress _address, String _message, Throwable _cause) {
		super(_address, _message, _cause);
	}

	public ConsistencyException(WorkingMemoryAddress _address, String _message) {
		super(_address, _message);
	}

	public ConsistencyException(WorkingMemoryAddress _address, Throwable _cause) {
		super(_address, _cause);
	}

	public ConsistencyException(WorkingMemoryAddress _address) {
		super(_address);
	}



}
