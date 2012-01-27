package analysis;

public class EpanechinikovKernel extends Kernel{


	public static void main(String[] args){
		
		EpanechinikovKernel k = new EpanechinikovKernel();
		System.out.println(k.apply(-2));
		System.out.println(k.apply(-1.5));
		System.out.println(k.apply(-1.25));
		System.out.println(k.apply(-1));
		System.out.println(k.apply(-0.75));
		
		System.out.println(k.apply(-0.5));
		System.out.println(k.apply(-0.25));
		System.out.println(k.apply(0));
		System.out.println(k.apply(0.25));
		System.out.println(k.apply(0.5));
		System.out.println(k.apply(0.75));
		System.out.println(k.apply(1));
		System.out.println(k.apply(1.25));
		System.out.println(k.apply(1.5));
		System.out.println(k.apply(2));
		
		
		//System.out.println(k.apply(15));
		
	}
	
	@Override
	public double apply(double input) {
		double value = (0.75)*(1-(input*input));
		if(Math.abs(value)<1 &&value>0){
			return value;
		}else{
			return 0;
		}
	}

}
