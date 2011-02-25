package manipulation.core.share.types;

/**
 * represents a polarcoordinate
 * 
 * @author ttoenige
 * 
 */
public class Polarcoordinate {
	private double r;
	private double phi;

	/**
	 * Constructor of the polarcoordiante
	 * 
	 * @param r
	 *            radius of the polarcoordinate
	 * @param phi
	 *            angle of the polarcoordinate
	 */
	public Polarcoordinate(double r, double phi) {
		super();
		this.r = r;
		this.phi = phi;
	}

	/**
	 * gets the radus of the coordinate
	 * 
	 * @return radius of the coordinate
	 */
	public double getR() {
		return r;
	}

	/**
	 * sets the radius of the polarcoordinate
	 * 
	 * @param r
	 *            radius value
	 */
	public void setR(double r) {
		this.r = r;
	}

	/**
	 * gets the angle of the coordinate
	 * 
	 * @return angle of the polarcoordinate
	 */
	public double getPhi() {
		return phi;
	}

	/**
	 * sets the angle of the polarcoordinate
	 * 
	 * @param phi
	 *            angle value
	 */
	public void setPhi(double phi) {
		this.phi = phi;
	}

	/**
	 * {@inheritDoc}
	 */
	@Override
	public String toString() {
		return "{POLARCOORDINATE: r: " + getR() + " phi: " + getPhi() + "}";
	}

}
