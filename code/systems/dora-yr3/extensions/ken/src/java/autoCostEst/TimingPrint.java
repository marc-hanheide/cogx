package autoCostEst;

import java.io.BufferedInputStream;
import java.io.FileInputStream;
import java.io.FileNotFoundException;
import java.io.IOException;
import java.io.ObjectInputStream;

import cast.architecture.ManagedComponent;

public class TimingPrint extends ManagedComponent{

	public void start(){
		ObjectInputStream in;
		try {
			in = new ObjectInputStream(
					new BufferedInputStream(new FileInputStream("timings.txt")));
			TimingsWrapper wrap = (TimingsWrapper)in.readObject();
			println(wrap.toString());
			println("done");
		} catch (FileNotFoundException e) {
			// TODO Auto-generated catch block
			println(e);
			e.printStackTrace();
		} catch (IOException e) {
			// TODO Auto-generated catch block
			println(e);
			e.printStackTrace();
		} catch (ClassNotFoundException e) {
			// TODO Auto-generated catch block
			println(e);
			e.printStackTrace();
		}


		
	}
}
