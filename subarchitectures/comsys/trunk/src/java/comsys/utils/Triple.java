package comsys.utils;

public class Triple<A, B, C> {

	public A first;
	public B second;
	public C third;
	
	public Triple() {
		first = null;
		second = null;
		third = null;
	}
	
	public Triple(A _first, B _second, C _third) {
		this.first = _first;
		this.second = _second;
		this.third = _third;
	}
	
	@Override
    public boolean equals(Object o) {
        if (!(o instanceof Triple))
            return false;
 
        Triple<?, ?, ?> other = (Triple) o;
        return equal(first, other.first) && equal(second, other.second) && equal(third, other.third);
    }
    
    private static final boolean equal(Object o1, Object o2) {
        if (o1 == null) {
            return o2 == null;
        }
        return o1.equals(o2);
    }
    
    @Override
    public int hashCode() {
        int hFirst = first == null ? 0 : first.hashCode();
        int hSecond = second == null ? 0 : second.hashCode();
        int hThird = third == null ? 0 : third.hashCode();
        return hFirst + 57 * hSecond + 111 * hThird;
    }

}
