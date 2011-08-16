package autoCostEst;

import java.io.Serializable;

public class TimingsWrapper implements Serializable{

	/**
	 * 
	 */
	private static final long serialVersionUID = 4036337794581556074L;
	private Timings[] times;

	public TimingsWrapper(Timings[] times) {
		this.times = times;
	}

	public Timings[] getTimings() {
		return times;
	}
	
	@Override
	public String toString(){
		String returnV="begin ";
		
		for(Timings time: times){
			returnV+=time.toString();
		}
		
		return returnV;
	}
}
