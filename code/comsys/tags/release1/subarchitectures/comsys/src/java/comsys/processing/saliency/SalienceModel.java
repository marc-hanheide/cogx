package comsys.processing.saliency;

import java.util.Hashtable;
import java.util.Vector;
import java.util.Enumeration;


public class SalienceModel extends Ice.ObjectImpl {

	Hashtable<SalientEntity, Float> distribution ;
	
	
	public SalienceModel() {
		distribution = new Hashtable<SalientEntity, Float>();
	}
	
	public void addSalientObjects(Vector<SalientEntity> objects) {
		
			float sum = 0.0f;
			for (Enumeration<Float> e = distribution.elements(); e.hasMoreElements();) {
				float score = e.nextElement().floatValue();
				sum += score;
			}
			for (Enumeration<SalientEntity> e = objects.elements(); e.hasMoreElements();) {
				SalientEntity entity = e.nextElement();
				float score = entity.getScore();
				sum += score;
			}

			// first renormalize the preexisting objects in distribution
			for (Enumeration<SalientEntity> o = distribution.keys() ; o.hasMoreElements() ;) {
				SalientEntity obj = o.nextElement() ;
				distribution.put(obj, new Float(obj.getScore()/sum));
			}
			
			// and add the new ones
			for (Enumeration<SalientEntity> o = objects.elements() ; o.hasMoreElements() ;) {
				SalientEntity obj = o.nextElement() ;
				distribution.put(obj, new Float(obj.getScore()/sum));
			}
	}
	
	public float getProbability (SalientEntity object) {
		if (distribution.containsKey(object)) {
			return distribution.get(object).floatValue();
		}
		return 0.0f;
	}
	
	public Hashtable<SalientEntity, Float> getDistribution() {
		return distribution;
	}
	
	public float getSaliencyOfDiscRef(String discref) {
		for (Enumeration<SalientEntity> e = distribution.keys(); e.hasMoreElements();) {
			SalientEntity entity = e.nextElement();
			if (entity.getClass().equals(DiscourseSalientEntity.class)) {
				DiscourseSalientEntity entity2 = (DiscourseSalientEntity) entity;
				if (entity2.nomvar.equals(discref)) {
					return distribution.get(entity).floatValue();
				}
			}
		}		
		log("WARNING: discourse referent \"" + discref +"\" does not exist in salience model");
		return 0;
	}
	
	public String toString() {
		String result = "[";
		for (Enumeration<SalientEntity> o = distribution.keys() ; o.hasMoreElements() ;) {
			SalientEntity obj = o.nextElement() ;
			result += "(" + obj.concept + ": " + distribution.get(obj) + ")";
			if (o.hasMoreElements()) {
				result += ", ";
			}
		}
		result += "]";
		return result;
	}
	
	private void log(String str) {
		System.out.println("[salience model] " + str);
	}
}
