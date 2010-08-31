/** _
 *  part of the ELM system.
 *  @author Dennis Stachowicz
 */

package elm.dbio;

import elm.types.ELMType;
import elm.types.TypeFactory;
import elm.types.TypeHierarchy;
import elm.types.TypeHierarchyFactory;
import elm.types.TypeHierarchyEdge;

import java.sql.Connection;
import java.sql.SQLException;
import java.sql.PreparedStatement;
import java.sql.ResultSet;
import java.sql.Statement;

import java.io.IOException;

import java.util.Vector;
import java.util.Set;
import java.util.Iterator;

public class TypeHierarchyIO<TypeT extends ELMType> implements TypeHierarchyReader<TypeT> {

    protected Connection        connection              = null;
    protected PreparedStatement ps_load_eth             = null;
    protected PreparedStatement ps_is_sub_type    	= null;
    protected PreparedStatement ps_get_sub_types    	= null;
    protected PreparedStatement ps_get_super_types    	= null;
    protected PreparedStatement ps_add_sat_edge         = null;
   
    protected String sourceTableName                    = null;
    protected String closureTableName                   = null;
    protected String typeColumn                         = null;
    protected String subtypeColumn                      = null;

    protected final TypeFactory<TypeT> tFactory;
    protected final TypeHierarchyFactory<TypeT> tHierarchyFactory;

    public TypeHierarchyIO(Connection c,
			   String sourceTableName,
			   String closureTableName,
			   String typeColumn,
			   String subtypeColumn,
			   TypeFactory<TypeT> tFactory,
			   TypeHierarchyFactory<TypeT> tHierarchyFactory) throws SQLException {

	connection = c;
	this.tFactory = tFactory;
	this.tHierarchyFactory = tHierarchyFactory;

	this.sourceTableName = sourceTableName;
	this.closureTableName = closureTableName;
	this.typeColumn = typeColumn;
	this.subtypeColumn = subtypeColumn;
                
	ps_load_eth = 
	    connection.prepareStatement("SELECT " + 
					typeColumn + ", " + 
					subtypeColumn +
					" FROM " + 
					sourceTableName
					);
 
	ps_is_sub_type = 
	    connection.prepareStatement("SELECT " + 
					typeColumn + ", " + 
					subtypeColumn +
					" FROM " + 
					sourceTableName +
					" WHERE " +
					typeColumn + " = ? AND " +
					subtypeColumn + " = ?"
					);
 
	ps_get_sub_types = 
	    connection.prepareStatement("SELECT " + 
					typeColumn + ", " + 
					subtypeColumn +
					" FROM " + 
					" WHERE " +
					typeColumn + " = ?"
					);

	ps_get_super_types = 
	    connection.prepareStatement("SELECT " + 
					typeColumn + ", " + 
					subtypeColumn +
					" FROM " +  
					" WHERE " +
					subtypeColumn + " = ?"
					);
	   
	ps_add_sat_edge = 
	    connection.prepareStatement("INSERT INTO " + 
					closureTableName +
					" (" + 
					typeColumn + ", " + 
					subtypeColumn + 
					") ( SELECT ?, ? " +	                             
	                                " WHERE NOT EXISTS " +
					" ( SELECT 1 FROM " + 
					closureTableName +
					" WHERE " + 
					typeColumn + " = ? AND " +
					subtypeColumn + " = ?)) ");
				      
    }


    protected synchronized TypeHierarchy<TypeT> readTypeHierarchy(ResultSet rs) 
	throws SQLException {
	  
	TypeHierarchy<TypeT> eth = tHierarchyFactory.getInstance();
	
	while (rs.next()) {
	    TypeT v1 = tFactory.getInstance(rs.getString(typeColumn));
	    TypeT v2 = tFactory.getInstance(rs.getString(subtypeColumn));
	    if (!eth.containsVertex(v1))
		eth.addVertex(v1);
	    if (!eth.containsVertex(v2))
		eth.addVertex(v2);
	    eth.addEdge(v1, v2);
	}

	return eth;
    }

    public synchronized TypeHierarchy<TypeT> loadSourceTypeHierarchy() throws SQLException {
	  
	return readTypeHierarchy(ps_load_eth.executeQuery());
    }
    
    /*
     *  flushes the eth_trans_closure table in the database 
     */
    protected synchronized void flushTable() throws SQLException {
    
	Statement s = connection.createStatement();	
	s.executeUpdate("DELETE FROM " + closureTableName);		
    }

    protected Vector<TypeT> getGraphSources(final TypeHierarchy<TypeT> eth) {

	Vector<TypeT> sources = new Vector<TypeT>();
	Set<TypeT> vertices = eth.vertexSet();
	Iterator<TypeT> iterator = vertices.iterator();
	
	while (iterator.hasNext()) {
	    TypeT t = iterator.next();
	    if (eth.inDegreeOf(t) == 0)
		sources.add(t);
	}
	
	return sources;    
    }

    /*
     *  will not change the object given as parameter but will rather 
     *  fill the eth_trans_closure table in the database 
     *  after flushing it
     */
    public synchronized void computeAndStoreTransitiveClosure(final TypeHierarchy<TypeT> eth) throws SQLException {
    
	Vector<TypeT> superTypes = new Vector<TypeT>();
	
	flushTable();
	
	for (TypeT t : getGraphSources(eth))	    
	    computeTCAndStoreHelper(eth, t, superTypes);
	
    }

    public void loadComputeTCStore() throws SQLException {
	TypeHierarchy<TypeT> h = loadSourceTypeHierarchy();
	computeAndStoreTransitiveClosure(h);
    }

    private void computeTCAndStoreHelper(final TypeHierarchy<TypeT> eth, 
					 TypeT currentType, 
					 Vector<TypeT> superTypes) throws SQLException {

	storeSuperTypesInfo(currentType, superTypes);
	superTypes.add(currentType);
				
	Set<TypeHierarchyEdge> edges = 
	    eth.outgoingEdgesOf(currentType);
	Iterator<TypeHierarchyEdge> iterator =
	    edges.iterator();
	while (iterator.hasNext())
	    computeTCAndStoreHelper(eth, eth.getEdgeTarget(iterator.next()), superTypes);
	
	superTypes.removeElementAt(superTypes.size()-1);
    }

    private void storeSuperTypesInfo(TypeT currentType, 
				     Vector<TypeT> superTypes) throws SQLException {
		
	ps_add_sat_edge.setString(2, currentType.getName());
	ps_add_sat_edge.setString(4, currentType.getName());
	for (TypeT t : superTypes) {
	    ps_add_sat_edge.setString(1, t.getName());
	    ps_add_sat_edge.setString(3, t.getName());
	    ps_add_sat_edge.executeUpdate();
	}	    
    }

    public synchronized boolean isSubType(final TypeT t1, final TypeT t2) throws SQLException {
	
	ps_is_sub_type.setString(1, t1.getName());
	ps_is_sub_type.setString(2, t2.getName());
	
	TypeHierarchy<TypeT> eth = readTypeHierarchy(ps_is_sub_type.executeQuery());
	
	return eth.containsEdge(t1, t2);
    }

    public synchronized Vector<TypeT> getSubTypes(final TypeT t1) throws SQLException {
	
	Vector<TypeT> stypes = new Vector<TypeT>();
	ps_get_sub_types.setString(1, t1.getName());
	
	ResultSet rs = ps_get_sub_types.executeQuery();
	while (rs.next())
	    stypes.add(tFactory.getInstance(rs.getString(subtypeColumn)));

	return stypes;
    }
    

    public synchronized Vector<TypeT> getSuperTypes(final TypeT t1) throws SQLException {
	
	Vector<TypeT> stypes = new Vector<TypeT>();
	ps_get_super_types.setString(1, t1.getName());
	
	ResultSet rs = ps_get_super_types.executeQuery();
	while (rs.next())
	    stypes.add(tFactory.getInstance(rs.getString(typeColumn)));

	return stypes;
    }    

}
