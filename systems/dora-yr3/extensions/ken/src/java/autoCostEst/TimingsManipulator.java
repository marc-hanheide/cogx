package autoCostEst;

import java.io.BufferedInputStream;
import java.io.BufferedOutputStream;
import java.io.File;
import java.io.FileInputStream;
import java.io.FileNotFoundException;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.ObjectInputStream;
import java.io.ObjectOutputStream;

import cast.architecture.ManagedComponent;

public class TimingsManipulator extends ManagedComponent {

	Timings[] timings;

	public void load() {
		try {
			ObjectInputStream in = new ObjectInputStream(
					new BufferedInputStream(new FileInputStream("timings.txt")));

			timings = ((TimingsWrapper) (in.readObject())).getTimings();

			in.close();

		} catch (IOException e) {
			// TODO Auto-generated catch block
			println(e);
			e.printStackTrace();
		} catch (ClassNotFoundException e) {
			// TODO Auto-generated catch block
			println(e);
			e.printStackTrace();

		}
		println("load finished");
	}

	public void fill() {
		for (Timings timing : timings) {

			for (int i = 0; i < 7; i++) {
				for (int j = 0; j < 24; j++) {
					for (int k = 0; k < 6; k++) {
						if (timing.getTimingFromArray(i, j, k) == 10) {
							timing.setTiming(i, j, k, 0);
						}
					}
				}
			}

		}
	}

	public void save() {
		ObjectOutputStream out;
		try {
			File file = new File("timings.txt");
			file.delete();
			out = new ObjectOutputStream(new BufferedOutputStream(
					new FileOutputStream("timings.txt")));
			TimingsWrapper wrap = new TimingsWrapper(timings);
			out.writeObject(wrap);
			out.close();
			println("saved");
		} catch (FileNotFoundException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		} catch (IOException e) {
			// TODO Auto-generated catch block

			e.printStackTrace();
		}
	}

	public void run() {
		load();
		fill();
		save();
	}

}
