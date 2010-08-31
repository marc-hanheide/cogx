/** _
 *  part of the ELM system.
 *  @author Dennis Stachowicz
 */

package elm.types;

public interface TypeHierarchyFactory<TypeT> {

    public TypeHierarchy<TypeT> getInstance();
}
