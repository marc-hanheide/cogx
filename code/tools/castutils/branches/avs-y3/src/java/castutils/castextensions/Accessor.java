package castutils.castextensions;

/**
 * 
 * @author nah
 * 
 * @param <EntryType>
 * @param <MemberType>
 */
public interface Accessor<EntryType,MemberType> {
	/**
	 * 
	 * @param _entry
	 * @return
	 */
	public abstract MemberType access(EntryType _entry);
}