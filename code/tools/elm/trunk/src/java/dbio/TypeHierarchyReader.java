/** _
 *  part of the ELM system.
 *  @author Dennis Stachowicz
 */

package elm.dbio;

import java.util.Vector;
import java.sql.SQLException;

public interface TypeHierarchyReader<TypeT> {

    public boolean       isSubType(final TypeT t1, final TypeT t2) throws SQLException;

    public Vector<TypeT> getSubTypes(final TypeT t1) throws SQLException;
    public Vector<TypeT> getSuperTypes(final TypeT t1) throws SQLException;
}