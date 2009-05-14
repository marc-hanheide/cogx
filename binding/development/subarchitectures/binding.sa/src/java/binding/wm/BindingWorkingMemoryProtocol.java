/**
 * 
 */
package binding.wm;

import BindingData.BINDING_QUERY_PREFIX_SOURCE_DATA;
import cast.architecture.subarchitecture.SubarchitectureWorkingMemoryProtocol;
import cast.cdl.WorkingMemoryAddress;

/**
 * Manages the language used for pull queries from the
 * SubarchitectureWorkingMemory. We may need a better long term solution
 * though! The key behaviours are as follows - pull from an indicated
 * working memory
 * 
 * 
 * Some duplication here. Need to determine what is actually needed in full system.
 * 
 * @author nah
 */
public class BindingWorkingMemoryProtocol
        extends
            SubarchitectureWorkingMemoryProtocol {

    public static class BindingWorkingMemoryPullQuery
            extends
                WorkingMemoryPullQuery {

        /**
         * @param __said
         * @param __count
         * @param _object
         * @param _type
         */
        public BindingWorkingMemoryPullQuery(String _srcID,
                String _srcSA, String __said, int __count,
                Object _object, WorkingMemoryQueryType _type) {
            super(_srcID, _srcSA, __said, __count, _object, _type);
        }

        /**
         * @param _pq
         */
        public BindingWorkingMemoryPullQuery(WorkingMemoryPullQuery _pq) {
            super(_pq.getSourceID(), _pq.getSourceSA(), _pq
                .getSubarchitectureID(), _pq.getCount(), _pq
                .getQueryObject(), _pq.getType());
        }

    }

    /**
     * Query working memory by particular source address binding.
     * 
     * @param _i
     * @return
     */
    public static String createSourceData(String _srcID,
                                          String _srcSA,
                                          String _targetSA,
                                          String _sourceDataType,
                                          WorkingMemoryAddress _sourceDataAddress) {

        // TODO double-check order across board

        return BINDING_QUERY_PREFIX_SOURCE_DATA.value + "\\" + _srcID
            + "\\" + _srcSA + "\\" + _targetSA + "\\" + _sourceDataType
            + "\\" + _sourceDataAddress.m_subarchitecture + "\\"
            + _sourceDataAddress.m_id;
    }

//    /**
//     * Query working memory to get all the binding instances that could
//     * match a set of features. Not necessarily ones that would bind to
//     * now, but ones that have no conflicting features.
//     * 
//     * @param _i
//     * @return
//     */
//    public static String createFeaturePointerQuery(String _srcID,
//                                                   String _srcSA,
//                                                   String _targetSA,
//                                                   String _type,
//                                                   WorkingMemoryAddress _fpwma) {
//
//        return FEATURE_SET_QUERY_PREFIX.value + "\\" + _srcID + "\\"
//            + _srcSA + "\\" + _targetSA + "\\" + _type + "\\"
//            + _fpwma.m_subarchitecture + "\\" + _fpwma.m_id;
//    }

    // public static WorkingMemoryPullQuery parseQuery(String _query)
    // throws SubarchitectureProcessException {
    //
    // // System.out
    // // .println("BindingWorkingMemoryProtocol.parseQuery():
    // // " + _query);
    //
    // if (_query.startsWith(BINDING_QUERY_PREFIX_SOURCE_DATA.value)) {
    // return extractBinding(_query);
    // }
    // else {
    // return SubarchitectureWorkingMemoryProtocol
    // .parseQuery(_query);
    // }
    // }

    // /**
    // * @param _query
    // * @return
    // */
    // private static WorkingMemoryPullQuery extractBinding(String
    // _query) {
    // // extract using same format as id query...
    // WorkingMemoryPullQuery pq = SubarchitectureWorkingMemoryProtocol
    // .extractID(_query);
    // // but create a different query object
    // return new BindingWorkingMemoryPullQuery(pq);
    // }

}
