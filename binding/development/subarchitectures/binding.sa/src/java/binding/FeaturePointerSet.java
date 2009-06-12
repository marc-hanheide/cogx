/**
 * 
 */
package binding;

import java.util.Comparator;
import java.util.TreeSet;

import BindingData.FeaturePointer;

/**
 * Simple subclass to hide comparator details.
 * 
 * @author nah
 */
public class FeaturePointerSet extends TreeSet<FeaturePointer> {

    /**
     * 
     */
    private static final long serialVersionUID = 208155245217826915L;

    private static class BFPComparator implements
            Comparator<FeaturePointer> {

        /*
         * (non-Javadoc)
         * 
         * @see java.util.Comparator#compare(java.lang.Object,
         *      java.lang.Object)
         */
        public int compare(FeaturePointer _bfp1,
                FeaturePointer _bfp2) {
            return _bfp1.address.compareTo(_bfp2.address);
        }

    }

    /**
     * 
     */
    public FeaturePointerSet() {
        super(new BFPComparator());
    }

}
