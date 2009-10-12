package comsys.processing.cca.abduction;

public class Pair<A, B> {
	
    public A fst;
    public B snd;
 
    public Pair() {
    	this.fst = null;
    	this.snd = null;
    }

    public Pair(A _fst, B _snd) {
        this.fst = _fst;
        this.snd = _snd;
    }
    
    public final boolean equals(Object o) {
        if (!(o instanceof Pair))
            return false;
 
        final Pair<?, ?> other = (Pair) o;
        return equal(fst, other.fst) && equal(snd, other.snd);
    }
    
    public static final boolean equal(Object o1, Object o2) {
        if (o1 == null) {
            return o2 == null;
        }
        return o1.equals(o2);
    }
 
    public int hashCode() {
        int hFst = fst == null ? 0 : fst.hashCode();
        int hSnd = snd == null ? 0 : snd.hashCode();
 
        return hFst + (57 * hSnd);
    }
}
