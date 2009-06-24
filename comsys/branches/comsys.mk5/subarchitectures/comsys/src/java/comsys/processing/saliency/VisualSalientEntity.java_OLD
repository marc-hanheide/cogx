package comsys.processing.saliency;

import org.cognitivesystems.common.autogen.Math.Vector3D;
// import visionarch.util.VisualAttributeMap;

public class VisualSalientEntity extends SalientEntity{
 
	Vector3D location ;
	String colour  = "" ;
	
	float DEFAULT_SCORE = 1.0f;
	
	public VisualSalientEntity (String concept, float score) {
		super(concept);
		this.score = score;
	}
	
	public VisualSalientEntity (String concept) {
		super(concept);
		score = DEFAULT_SCORE;
	}
	
	public void setLocation(Vector3D location) {
		this.location = location;
	}
	
	/**
	public void setColour (int colour) { 
		try {
			this.colour = VisualAttributeMap.attribute(new Integer(colour));
		}
		catch (Exception e) {
			e.printStackTrace();
		}
	}
	*/
	
	public Vector3D getLocation () { return location ; }
	
	public String toString() {
		String result = concept + ": coordinates = (" + location.m_x + "," + location.m_y + "," + location.m_z + ")";
		if (!colour.equals("")) {
			result += ", colour = " + colour;
		}
		return result;
	}
	
	public boolean isWellFormed() {
		return ((!concept.equals("")) && (location != null)); 
	}
}