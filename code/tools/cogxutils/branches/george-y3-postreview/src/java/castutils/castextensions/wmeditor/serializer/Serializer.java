package castutils.castextensions.wmeditor.serializer;

import java.io.Reader;

public interface Serializer {
	public Object load(String str);
	public Object load(Reader reader);
	public String dump(Object obj);
}
