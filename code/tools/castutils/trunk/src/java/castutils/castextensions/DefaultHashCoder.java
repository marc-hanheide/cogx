package castutils.castextensions;

public class DefaultHashCoder<T> implements HashCoder<T> {

	public int hashCode(T _t) {
		return _t.hashCode();
	}

}
