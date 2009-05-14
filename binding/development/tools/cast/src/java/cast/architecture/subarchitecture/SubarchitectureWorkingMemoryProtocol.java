/*
 * CAST - The CoSy Architecture Schema Toolkit
 *
 * Copyright (C) 2006-2007 Nick Hawes
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public License
 * as published by the Free Software Foundation; either version 2.1 of
 * the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
 * 02110-1301 USA
 *
 */

/**
 * 
 */
package cast.architecture.subarchitecture;

import java.util.NoSuchElementException;
import java.util.StringTokenizer;

import cast.architecture.subarchitecture.SubarchitectureWorkingMemoryProtocol.WorkingMemoryPullQuery.WorkingMemoryQueryType;
import cast.cdl.*;

/**
 * Manages the language used for pull queries from the
 * SubarchitectureWorkingMemory. We may need a better long term solution
 * though! The key behaviours are as follows - pull from an indicated
 * working memory
 * 
 * @author nah
 */
public class SubarchitectureWorkingMemoryProtocol {

    public static class WorkingMemoryPullQuery {

        public enum WorkingMemoryQueryType {
            TYPE, ID, EXISTS, ID_ARRAY, OVERWRITE_COUNT, OTHER
        };

        // ID of the sub-architecture
        private String m_saID;
        // max number of results requested
        private int m_count;
        // query object
        private Object m_queryObject;
        // and the type
        private WorkingMemoryQueryType m_type;
        private String m_srcID;
        private String m_srcSA;

        /**
         * @return the srcID
         */
        public String getSourceID() {
            return m_srcID;
        }

        /**
         * @return the srcSA
         */
        public String getSourceSA() {
            return m_srcSA;
        }

        /**
         * @param __said
         * @param __count
         * @param _object
         */
        public WorkingMemoryPullQuery(String _srcID, String _srcSA,
                String __said, int __count, Object _object,
                WorkingMemoryQueryType _type) {
            m_srcID = _srcID;
            m_srcSA = _srcSA;

            m_saID = __said;
            m_count = __count;
            m_queryObject = _object;
            m_type = _type;
        }

        public WorkingMemoryQueryType getType() {
            return m_type;
        }

        public int getCount() {
            return m_count;
        }

        public Object getQueryObject() {
            return m_queryObject;
        }

        public String getSubarchitectureID() {
            return m_saID;
        }

    }

    protected static final String ID_PREFIX = ID_QUERY_PREFIX.value;
    protected static final String EXISTS_PREFIX =
            EXISTS_QUERY_PREFIX.value;
    
    protected static final String OVERWRITE_COUNT_PREFIX =
        OVERWRITE_COUNT_QUERY_PREFIX.value;

    
    protected static final String ID_ARRAY_PREFIX =
            ID_ARRAY_QUERY_PREFIX.value;
    protected static final String TYPE_PREFIX = TYPE_QUERY_PREFIX.value;

    /**
     * Query working memory by particular index.
     * 
     * @param _i
     * @return
     */
    public static String createIDQuery(String _srcID,
                                       String _srcSA,
                                       String _subarch,
                                       String _id) {
        return ID_PREFIX + "\\" + _srcID + "\\" + _srcSA + "\\"
            + _subarch + "\\" + _id;
    }

    public static String createExistsQuery(String _srcID,
                                           String _srcSA,
                                           String _subarch,
                                           String _id) {
        return EXISTS_PREFIX + "\\" + _srcID + "\\" + _srcSA + "\\"
            + _subarch + "\\" + _id;
    }

    public static String createOverwriteCountQuery(String _srcID,
                                           String _srcSA,
                                           String _subarch,
                                           String _id) {
        return OVERWRITE_COUNT_PREFIX + "\\" + _srcID + "\\" + _srcSA + "\\"
            + _subarch + "\\" + _id;
    }
    
    /**
     * Query working memory by a collection of indices.
     * 
     * @param _is
     * @return
     */
    public static String createIDArrayQuery(String _srcID,
                                            String _srcSA,
                                            String _subarch,
                                            String[] _ids) {
        String s =
                ID_ARRAY_PREFIX + "\\" + _srcID + "\\" + _srcSA + "\\"
                    + _subarch + "\\";
        for (int i = 0; i < _ids.length; i++) {
            s += _ids[i] + "\\";
        }
        return s;
    }

    /**
     * Query working memory by ontological type. Restrict number of
     * results. Starts from most recent.
     * 
     * @param _type
     * @param _count
     * @return
     */
    public static String createTypeQuery(String _srcID,
                                         String _srcSA,
                                         String _subarch,
                                         String _type,
                                         int _count) {
        return TYPE_PREFIX + "\\" + _srcID + "\\" + _srcSA + "\\"
            + _subarch + "\\" + _count + "\\" + _type;
    }

    public static WorkingMemoryPullQuery parseQuery(String _query)
            throws SubarchitectureProcessException {

        // System.out
        // .println("SubarchitectureWorkingMemoryProtocol.parseQuery():
        // " + _query);

        if (_query.startsWith(ID_PREFIX)) {
            return extractID(_query);
        }
        else if (_query.startsWith(EXISTS_PREFIX)) {
            return extractExists(_query);
        }
        else if (_query.startsWith(OVERWRITE_COUNT_PREFIX)) {
            return extractOverwriteCount(_query);
        }
        else if (_query.startsWith(ID_ARRAY_PREFIX)) {
            return extractIDArray(_query);
        }
        else if (_query.startsWith(TYPE_PREFIX)) {
            return extractType(_query);
        }
        else {
            return extractUnknown(_query);
        }

    }

    protected static WorkingMemoryPullQuery extractUnknown(String _query)
            throws SubarchitectureProcessException {

        // System.out
        // .println("SubarchitectureWorkingMemoryProtocol.extractID()");
        // System.out.println(_query);

        try {
            StringTokenizer tokenizer =
                    new StringTokenizer(_query, "\\");
            // skip prefix token
            tokenizer.nextToken();

            // assume these 3 are ok... the rest can be unknown
            String srcID = tokenizer.nextToken();
            String srcSA = tokenizer.nextToken();
            String targetSA = tokenizer.nextToken();

            return new WorkingMemoryPullQuery(srcID, srcSA, targetSA,
                0, _query, WorkingMemoryQueryType.OTHER);

        }
        catch (NoSuchElementException e) {
            throw new SubarchitectureProcessException(
                "Unknown memory protocol for query: " + _query, e);
        }
    }

    protected static WorkingMemoryPullQuery extractID(String _query) {

        // System.out
        // .println("SubarchitectureWorkingMemoryProtocol.extractID()");
        // System.out.println(_query);

        StringTokenizer tokenizer = new StringTokenizer(_query, "\\");
        // skip prefix token
        tokenizer.nextToken();

        String srcID = tokenizer.nextToken();
        String srcSA = tokenizer.nextToken();
        String subarch = tokenizer.nextToken();
        String intString = tokenizer.nextToken();

        return new WorkingMemoryPullQuery(srcID, srcSA, subarch, 0,
            intString, WorkingMemoryQueryType.ID);

    }

    protected static WorkingMemoryPullQuery extractExists(String _query) {

        // System.out
        // .println("SubarchitectureWorkingMemoryProtocol.extractID()");
        // System.out.println(_query);

        StringTokenizer tokenizer = new StringTokenizer(_query, "\\");
        // skip prefix token
        tokenizer.nextToken();

        String srcID = tokenizer.nextToken();
        String srcSA = tokenizer.nextToken();
        String subarch = tokenizer.nextToken();
        String intString = tokenizer.nextToken();

        return new WorkingMemoryPullQuery(srcID, srcSA, subarch, 0,
            intString, WorkingMemoryQueryType.EXISTS);

    }

    protected static WorkingMemoryPullQuery extractOverwriteCount(String _query) {

        // System.out
        // .println("SubarchitectureWorkingMemoryProtocol.extractID()");
        // System.out.println(_query);

        StringTokenizer tokenizer = new StringTokenizer(_query, "\\");
        // skip prefix token
        tokenizer.nextToken();

        String srcID = tokenizer.nextToken();
        String srcSA = tokenizer.nextToken();
        String subarch = tokenizer.nextToken();
        String intString = tokenizer.nextToken();

        return new WorkingMemoryPullQuery(srcID, srcSA, subarch, 0,
            intString, WorkingMemoryQueryType.OVERWRITE_COUNT);

    }
    
    protected static WorkingMemoryPullQuery extractIDArray(String _query) {
        StringTokenizer tokenizer = new StringTokenizer(_query, "\\");
        // skip prefix token
        tokenizer.nextToken();

        String srcID = tokenizer.nextToken();
        String srcSA = tokenizer.nextToken();
        String subarch = tokenizer.nextToken();

        String ia[] = new String[tokenizer.countTokens()];
        int i = 0;

        while (tokenizer.hasMoreTokens()) {
            ia[i++] = tokenizer.nextToken();
        }

        return new WorkingMemoryPullQuery(srcID, srcSA, subarch, 0, ia,
            WorkingMemoryQueryType.ID_ARRAY);

    }

    protected static WorkingMemoryPullQuery extractType(String _query) {

        // STRING_PREFIX + "\\" + _subarch + "\\" + _count + "\\" + _s;

        StringTokenizer tokenizer = new StringTokenizer(_query, "\\");

        // skip prefix token
        tokenizer.nextToken();

        String srcID = tokenizer.nextToken();
        String srcSA = tokenizer.nextToken();

        String subarch = tokenizer.nextToken();

        String count = tokenizer.nextToken();

        String typeString = tokenizer.nextToken();

        return new WorkingMemoryPullQuery(srcID, srcSA, subarch,
            Integer.parseInt(count), typeString,
            WorkingMemoryQueryType.TYPE);
    }

    public static void main(String[] args) {
    // String query = memoryQuery(999);
    // System.out.println(query);
    // int i = extractInt(query);
    // System.out.println(i);

    // String query = memoryQuery(new int []{999,1, 22, 345});
    // System.out.println(query);
    // int ia[] = extractIntArray(query);
    // for (int i = 0; i < ia.length; i++) {
    // System.out.println(ia[i]);
    // }

    // String query = memoryQuery("blah");
    // System.out.println(query);
    // String s = extractString(query);
    // System.out.println(s);

    }

    /**
     * @param _queryObject
     * @return
     * @throws SubarchitectureProcessException
     * @throws SubarchitectureProcessException
     */
    public static String createQuery(WorkingMemoryPullQuery _queryObject)
            throws SubarchitectureProcessException {
        if (_queryObject.getType() == WorkingMemoryQueryType.ID) {
            return createIDQuery(_queryObject.getSourceID(),
                _queryObject.getSourceSA(), _queryObject
                    .getSubarchitectureID(), (String) _queryObject
                    .getQueryObject());
        }
        else if (_queryObject.getType() == WorkingMemoryQueryType.EXISTS) {
            return createExistsQuery(_queryObject.getSourceID(),
                _queryObject.getSourceSA(), _queryObject
                    .getSubarchitectureID(), (String) _queryObject
                    .getQueryObject());
        }
        else if (_queryObject.getType() == WorkingMemoryQueryType.OVERWRITE_COUNT) {
            return createOverwriteCountQuery(_queryObject.getSourceID(),
                _queryObject.getSourceSA(), _queryObject
                    .getSubarchitectureID(), (String) _queryObject
                    .getQueryObject());
        }
        else if (_queryObject.getType() == WorkingMemoryQueryType.ID_ARRAY) {
            return createIDArrayQuery(_queryObject.getSourceID(),
                _queryObject.getSourceSA(), _queryObject
                    .getSubarchitectureID(), (String[]) _queryObject
                    .getQueryObject());
        }
        else if (_queryObject.getType() == WorkingMemoryQueryType.TYPE) {
            return createTypeQuery(_queryObject.getSourceID(),
                _queryObject.getSourceSA(), _queryObject
                    .getSubarchitectureID(), (String) _queryObject
                    .getQueryObject(), _queryObject.getCount());
        }
        else if (_queryObject.getType() == WorkingMemoryQueryType.OTHER) {
            return (String) _queryObject.getQueryObject();
        }

        throw new SubarchitectureProcessException(
            "Unknown memory protocol for query: "
                + _queryObject.getType());
    }

}
