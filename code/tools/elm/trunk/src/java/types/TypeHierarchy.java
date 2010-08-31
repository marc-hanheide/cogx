/** _
 *  part of the ELM system.
 *  @author Dennis Stachowicz
 */

package elm.types;

import org.jgrapht.graph.DefaultDirectedGraph;

public class TypeHierarchy<T> extends DefaultDirectedGraph<T, TypeHierarchyEdge> {

    public TypeHierarchy() {
	super(TypeHierarchyEdge.class);
    }

}
