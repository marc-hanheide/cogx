package analysis;

public class GaussianKernel extends Kernel {

	@Override
	public double apply(double input) {
		double val =Math.pow((1/Math.sqrt(2*Math.PI)),(-.5*input*input));
		return val;
	}

}
