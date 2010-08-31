/** _
 *  part of the ELM system.
 *  @author Dennis Stachowicz
 */

package elm.types;

public class ELMType {

    protected String typename = null;
    
    public ELMType(String typename) {
	this.typename = typename;
    }

    public String getName() {
	return typename;
    }
    
    public boolean isName(String s) {
	return typename.equals(s);
    }

    public String toString() {
	return getName();
    }

    public boolean equals(Object o) {
	
	if (o instanceof ELMType) {
	    ELMType e = (ELMType) o;
	    return typename.equals(e.typename);
	}
	else if (o instanceof String) {
	    String s = (String) o;
	    return typename.equals(s);
	}
	else 
	    return false;
    }

    public int hashCode() {
	return typename.hashCode();
    }

   
}
