package comsys.processing.saliency;

public class WeightedWord {
		
		String word;
		float weight;
		
		public WeightedWord (String word, float weight) {
			this.word = word;
			this.weight = weight;
		}
		
		public void modifyWord (String word) {
			this.word = word;
		}
		
		public void modifyWeight (float weight) {
			this.weight = weight;
		}
		
		public float getWeight() {
			return weight;
		}
		
		public String getWord() {
			return word;
		}

}
