package comsys.datastructs.testData;

import java.util.Vector;
import comsys.utils.DialogueMoveUtils;
import comsys.datastructs.comsysEssentials.MoveType;


public class VisualGroundingTestData extends TestData {

	
	public class VisualObject {
		
		String label = "";
		String color = "";
		String size = "";
		
		public VisualObject(String label) {
			this.label = label;
		}
		
		public String getLabel() {
			return label;
		}
		
		public String getColor() {
			return color;
		}
		
		public String getSize() {
			return size;
		}
		
		public void setLabel(String label) {
			this.label = label;
		}
		
		public void setColor(String color) {
			this.color = color;
		}
		
		public void setSize(String size) {
			this.size = size;
		}
	}
	
	boolean logging = false;
	
	String utterance;
	Vector<VisualObject> visualscene;
	boolean grounding;
	String referent;
	
	public VisualGroundingTestData() {
		super();
		visualscene = new Vector<VisualObject>();
		grounding = false;
		referent = "";
	}
	
	public VisualGroundingTestData(String utterance, VisualObject object) {
		this.utterance = utterance;
		visualscene = new Vector<VisualObject>();
		grounding = false;
		referent = "";
	}
	
	
	public String getString() {
		return utterance;
	}
	
	public VisualObject addVisualObject(String label) {
		VisualObject object = new VisualObject(label);
		visualscene.add(object);
		return object;
	}
	
	public void setShouldBeGrounded(boolean grounding) {
		this.grounding = grounding;
	}
	
	public boolean shouldBeGrounded() {
		return grounding;
	}
	
	public void setReferent(String referent) {
		this.referent = referent;
	}
	
	public String getReferent() {
		return referent;
	}
	
	
	
	public Vector<VisualObject> getVisualScene() {
		return visualscene;
	}
	public void setString(String utterance) {
		this.utterance = utterance;
	}
	
	private void log(String str) {
		if (logging) System.out.println("[VisualGroundingData] " +  str);
	}
}
