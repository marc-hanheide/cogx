package castutils.castextensions;

/**
 * Gets a hash for the input object. Used when a provided object does not do the right thing for us.
 * 
 * @author nah
 *
 * @param <T>
 */
public interface HashCoder<T> {
	int hashCode(T _t);
}
