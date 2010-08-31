/** _
 *  part of the ELM system.
 *  @author Dennis Stachowicz
 */

package elm.types;

public interface TypeFactory<TypeT> {

    public TypeT getInstance(String typename);
}
