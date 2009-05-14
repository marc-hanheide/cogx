/*
 * Comedian example code to demonstrate CAST functionality.
 *
 * Copyright (C) 2006-2007 Nick Hawes
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public License
 * as published by the Free Software Foundation; either version 2.1 of
 * the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
 * 02110-1301 USA
 *
 */

/**
 * 
 */
package comedyarch.subarchitectures.stage;

import java.util.Hashtable;
import java.util.ListIterator;
import java.util.Vector;

import cast.architecture.abstr.ChangeFilterFactory;
import cast.architecture.abstr.WorkingMemoryChangeReceiver;
import cast.architecture.subarchitecture.PrivilegedManagedProcess;
import cast.architecture.subarchitecture.SubarchitectureProcessException;
import cast.cdl.TaskOutcome;
import cast.cdl.WorkingMemoryChange;
import cast.cdl.WorkingMemoryOperation;
import cast.cdl.guitypes.FAT;
import cast.cdl.guitypes.FILLED;
import cast.cdl.guitypes.NONE;
import cast.core.data.CASTData;

import comedyarch.autogen.Joke;
import comedyarch.global.ComedyGoals;

/**
 * @author nah
 */
public class FunnyManGUI extends PrivilegedManagedProcess {

	private final int X = 0;

	private final int Y = 1;

	private final int Z = 2;

	// number of moving drawing primitives
	private final int numPrimitives2D = 4;

	private final int numPrimitives3D = 3;

	// maximum velocity of a 2D drawing primitive (actually sqrt(2)..)
	private final double velMax2D = 5.;

	// maximum velocity of a 3D drawing primitive (actually sqrt(2)..)
	private final double velMax3D = 0.02;

	// time to wait between steps [ms]
	// 50 ms corresponds to a frame rate of 20 Hz
	private final int stepTime = 50;

	// size of the field where 2D drawing primitives move around
	// note that 3D primitives just move within the unit cube
	private final double[] min2D;

	private final double[] max2D;

	// positions of 2D drawing primitives
	private double[][] pos2D;

	// velocities of 2D drawing primitives
	private double[][] vel2D;

	// positions of 3D drawing primitives
	private double[][] pos3D;

	// velocities of 3D drawing primitives
	private double[][] vel3D;

	private final int imgWidth = 320;

	private final int imgHeight = 240;

	byte[] rgbImage;

	// Hashtable used to record the tasks we want to carry out
	private Hashtable<String, CASTData> m_proposedProcessing;

	private Vector<CASTData> m_setups;

	/**
	 * @param _id
	 */
	public FunnyManGUI(String _id) {
		super(_id);

		min2D = new double[2];
		max2D = new double[2];
		pos2D = new double[numPrimitives2D][2];
		vel2D = new double[numPrimitives2D][2];
		min2D[X] = 0.;
		min2D[Y] = 0.;
		max2D[X] = 400.;
		max2D[Y] = 300.;
		for (int i = 0; i < numPrimitives2D; i++) {
			for (int j = X; j <= Y; j++) {
				// all primitives start out in the center
				pos2D[i][j] = (min2D[j] + max2D[j]) / 2.;
				// with random velocities
				vel2D[i][j] = velMax2D * 2 * (Math.random() - 0.5);
			}
		}

		// set up 3D motions
		pos3D = new double[numPrimitives3D][3];
		vel3D = new double[numPrimitives3D][3];
		for (int i = 0; i < numPrimitives3D; i++) {
			for (int j = X; j <= Z; j++) {
				// all primitives start out in the center of the unit
				// cube
				pos3D[i][j] = 0.5;
				// with random velocities
				vel3D[i][j] = velMax3D * 2 * (Math.random() - 0.5);
			}
		}

		// create our image to display
		rgbImage = createImage(imgWidth, imgHeight);

//		setOntology(ComedyOntologyFactory.getOntology());
		m_proposedProcessing = new Hashtable<String, CASTData>();
		m_setups = new Vector<CASTData>();

		// addChangeFilter(ComedyOntology.JOKE_TYPE,
		// WorkingMemoryOperation.ADD);

	}

	/*
	 * (non-Javadoc)
	 * 
	 * @see cast.architecture.abstr.WorkingMemoryReaderProcess#start()
	 */
	@Override
	public void start() {
		super.start();
		try {
			addChangeFilter(ChangeFilterFactory.createLocalTypeFilter(
					Joke.class, WorkingMemoryOperation.ADD),
					new WorkingMemoryChangeReceiver() {

						public void workingMemoryChanged(
								WorkingMemoryChange _wmc) {
							newJokeAdded(_wmc);
						}
					});
		} catch (SubarchitectureProcessException e) {
			e.printStackTrace();
		}
	}

	protected void redrawGraphics2D() {
		final double l = 100.; // length of line, size of rectangle

		// draw image
		drawRGBImage(imgWidth, imgHeight, rgbImage, NONE.value);

		// draw moving thingies
		drawPoint2D(pos2D[0][X], pos2D[0][Y], 255, 255, 0, FAT.value);
		drawLine2D(pos2D[1][X], pos2D[1][Y], pos2D[1][X] + l, pos2D[1][Y] + l,
				0, 255, 0, FAT.value);
		drawRect2D(pos2D[2][X], pos2D[2][Y], pos2D[2][X] + l, pos2D[2][Y] + l,
				255, 0, 0, NONE.value);
		drawPolygon2D(createPolyX(5), createPolyY(5), 255, 0, 255, FILLED.value);
	}

	protected void redrawGraphics3D() {
		final double l = 0.1; // length of line

		// draw ground plane
		drawLine3D(0., 0., 0., 1., 0., 0., 128, 128, 0, NONE.value);
		drawLine3D(1., 0., 0., 1., 1., 0., 128, 128, 0, NONE.value);
		drawLine3D(1., 1., 0., 0., 1., 0., 128, 128, 0, NONE.value);
		drawLine3D(0., 1., 0., 0., 0., 0., 128, 128, 0, NONE.value);
		// and z-axis
		drawLine3D(0., 0., 0., 0., 0., 1., 128, 128, 0, NONE.value);

		// draw moving thingies
		drawPoint3D(pos3D[0][X], pos3D[0][Y], pos3D[0][Z], 255, 255, 0,
				FAT.value);
		drawLine3D(pos3D[1][X], pos3D[1][Y], pos3D[1][Z], pos3D[1][X] + l,
				pos3D[1][Y] + l, pos3D[1][Z] + l, 0, 255, 0, FAT.value);
		drawCat(pos3D[2][X], pos3D[2][Y], pos3D[2][Z]);
	}

	protected void redrawGraphicsText() {
		printText("status:\t\t\trunning\n");
		printText("\n");
		printText("--- drawing primitives: -----------------------------------\n");
		printText("number 2D:\t\t\t" + numPrimitives2D + "\n");
		printText("range 2D:\t\t\t(" + (int) min2D[X] + " " + (int) min2D[Y]
				+ ") to (" + (int) max2D[X] + " " + (int) max2D[Y] + ")\n");
		printText("number 3D:\t\t\t" + numPrimitives3D + "\n");
		printText("range 3D:\t\t\t(0 0 0) to (1 1 1)\n");
		printText("\n");
		printText("--- positions 2D: -----------------------------------------\n");
		for (int i = 0; i < numPrimitives2D; i++)
			printText("position " + i + ":\t\t(" + (int) pos2D[i][X] + " "
					+ (int) pos2D[i][Y] + ")\n");
		printText("\n");
		printText("--- positions 3D: -----------------------------------------\n");
		for (int i = 0; i < numPrimitives3D; i++)
			printText("position " + i + ":\t\t(" + pos3D[i][X] + " "
					+ pos3D[i][Y] + " " + pos3D[i][Z] + ")\n");
	}

	/**
	 * Create a black/white checkerboard image of size w*h.
	 */
	private byte[] createImage(int w, int h) {
		byte[] rgb = new byte[3 * w * h];
		int cnt = 0;
		for (int i = 0; i < h; i++)
			for (int j = 0; j < w; j++)
				if ((i / 40 + j / 40) % 2 == 0) {
					rgb[cnt] = (byte) 255;
					cnt++;
					rgb[cnt] = (byte) 255;
					cnt++;
					rgb[cnt] = (byte) 255;
					cnt++;
				} else {
					rgb[cnt] = (byte) 0;
					cnt++;
					rgb[cnt] = (byte) 0;
					cnt++;
					rgb[cnt] = (byte) 0;
					cnt++;
				}
		return rgb;
	}

	/**
	 * Create the x co-ordinate values for an n-sided polygon, already adding
	 * the position offset for animation.
	 */
	private double[] createPolyX(int n) {
		final double rad = 50.;
		double[] x = new double[n];
		for (int i = 0; i < n; i++)
			x[i] = pos2D[3][X] + rad * Math.cos(i * 2 * Math.PI / n);
		return x;
	}

	/**
	 * Create the y co-ordinate values for an n-sided polygon, already adding
	 * the position offset for animation.
	 */
	private double[] createPolyY(int n) {
		final double rad = 50.;
		double[] y = new double[n];
		for (int i = 0; i < n; i++)
			y[i] = pos2D[3][Y] + rad * Math.sin(i * 2 * Math.PI / n);
		return y;
	}

	/**
	 * Draw a cat :)
	 */
	private void drawCat(double x, double y, double z) {
		double l = 0.05;
		double[][] cat = new double[12][3];

		cat[0][X] = 0.6;
		cat[0][Y] = 0.0;
		cat[0][Z] = 0.0;
		cat[1][X] = 2.0;
		cat[1][Y] = 0.0;
		cat[1][Z] = 1.0;
		cat[2][X] = 2.0;
		cat[2][Y] = 0.0;
		cat[2][Z] = 4.0;
		cat[3][X] = 1.0;
		cat[3][Y] = 0.0;
		cat[3][Z] = 3.0;
		for (int i = 4; i < 8; i++) {
			// mirror around y-axis
			cat[i][X] = -cat[7 - i][X];
			cat[i][Y] = cat[7 - i][Y];
			cat[i][Z] = cat[7 - i][Z];
		}
		cat[8][X] = 0.4;
		cat[8][Y] = 0.0;
		cat[8][Z] = 1.8;
		cat[9][X] = 1.4;
		cat[9][Y] = 0.0;
		cat[9][Z] = 2.0;
		cat[10][X] = -0.4;
		cat[10][Y] = 0.0;
		cat[10][Z] = 1.8;
		cat[11][X] = -1.4;
		cat[11][Y] = 0.0;
		cat[11][Z] = 2.0;

		for (int i = 0; i < 8; i++) {
			int j = (i == 7 ? 0 : i + 1);
			drawLine3D(x + l * cat[i][X], y + l * cat[i][Y], z + l * cat[i][Z],
					x + l * cat[j][X], y + l * cat[j][Y], z + l * cat[j][Z], 0,
					0, 255, NONE.value);
		}
		for (int i = 8; i < 12; i += 2) {
			int j = i + 1;
			drawLine3D(x + l * cat[i][X], y + l * cat[i][Y], z + l * cat[i][Z],
					x + l * cat[j][X], y + l * cat[j][Y], z + l * cat[j][Z], 0,
					0, 255, NONE.value);
		}
	}

	/*
	 * (non-Javadoc)
	 * 
	 * @see cast.architecture.subarchitecture.ManagedProcess#taskAdopted(java.lang.String)
	 */
	@Override
	protected void taskAdopted(String _goalID) {
		// wooo! we're allowed to act

		// get the data we stored for this goal...
		CASTData data = m_proposedProcessing.remove(_goalID);

		// if we have stored this goal earlier
		if (data != null) {
			// we could call quip(data) directly, but let's queue the
			// data instead... it's always good to build some suspense
			m_setups.add(data);
		} else {
			println("oh, this is my goal, but I have no data: " + _goalID);
		}

		// and now we're finished, tell the goal manager that the task
		// is over successfully (assuming it is... naughty!)
		try {
			taskComplete(_goalID, TaskOutcome.PROCESSING_COMPLETE_SUCCESS);
		} catch (SubarchitectureProcessException e) {
			e.printStackTrace();
		}
	}

	private String generatePunchline(String _setup) {
		return "A stick!";
	}

	/**
	 * @param _data
	 */
	private void quip(CASTData _data) {
		Joke jk = (Joke) _data.getData();
		// work out a smarty-pants punchline
		String punchline = generatePunchline(jk.m_setup);
		// time it right
		log("*cough*");
		jk.m_punchline = punchline;
		// now write this back into working memory
		try {
			overwriteWorkingMemory(_data.getID(), jk);
		} catch (SubarchitectureProcessException e) {
			e.printStackTrace();
		}

	}

	/*
	 * (non-Javadoc)
	 * 
	 * @see cast.architecture.subarchitecture.ManagedProcess#taskRejected(java.lang.String)
	 */
	@Override
	protected void taskRejected(String _goalID) {
		println(":(");
		m_proposedProcessing.remove(_goalID);
	}

	/**
	 * @param _wmc
	 * @param i
	 */
	private void newJokeAdded(WorkingMemoryChange _wmc) {
		try {
			// get the id of the working memory entry
			String id = _wmc.m_address.m_id;

			// get the data from working memory and store it
			// with its id
			CASTData jokeData = getWorkingMemoryEntry(id);

			// test
			// CASTTypedData[] data =
			// getWorkingMemoryEntries(new String[]{id});

			//                
			// CASTTypedDataWithID[] jokes =
			// getWorkingMemoryEntries(
			// _wmc[i].m_type, 0);
			//
			// CASTTypedDataWithID jokeData = jokes[0];

			// endtest

			// because we've checked the ontological type above,
			// this should be safe
			Joke jk = (Joke) jokeData.getData();

			// now see if we can provide a punchline
			if (jk.m_punchline.equals("")) {
				// in this case we need to propose a task to do
				// some processsing

				// get a new id for the task
				String taskID = newTaskID();

				// store the data we want to process for
				// later... we could just store the id in
				// working memory if we wanted to, I guess it
				// depends on storage/transport tradeoffs
				m_proposedProcessing.put(taskID, jokeData);

				// then ask for permission
				proposeInformationProcessingTask(taskID,
						ComedyGoals.ADD_PUNCHLINE_TASK);

				// then immediately retract!
				// retractInformationProcessingTask(taskID);

			} else {
				// nothing to do then :(
			}

		} catch (SubarchitectureProcessException e) {
			println(e.getLocalizedMessage());
			e.printStackTrace();
		}
	}

	/*
	 * (non-Javadoc)
	 * 
	 * @see cast.core.components.CASTComponent#runComponent()
	 */
	@Override
	public void runComponent() {

		// HACK otherwise GUI crashes!
		sleepProcess(1000);

		// try {

		// TaskDescription desc = new
		// TaskDescription(ComedyGoals.ADD_PUNCHLINE_TASK);
		// registerTaskDescriptions(new TaskDescription[]{desc});

		while (m_status == ProcessStatus.RUN) {
			// do nothing for a while
			// waitForNotifications();

			sleepProcess(stepTime);

			// must check we're still running after sleep!
			if (m_status == ProcessStatus.RUN) {

				doProcessingStuff();

				doGUIStuff();

			}
		}
		// }
		// catch (InterruptedException e) {
		// e.printStackTrace();
		// }
		// catch (SubarchitectureProcessException e) {
		// e.printStackTrace();
		// }
	}

	/**
	 * 
	 */
	private void doProcessingStuff() {
		// check (synchronised) joke queue
		ListIterator<CASTData> i = m_setups.listIterator();

		// see what's in there
		while (i.hasNext()) {
			// lock from external access
			lockProcess();

			// quick, tell the punchline
			quip(i.next());

			// remove data from queue
			i.remove();

			// let other stuff happen if necessary
			unlockProcess();
		}
	}

	/**
	 * 
	 */
	private void doGUIStuff() {
		redrawGraphicsNow();

		// update 2D positions
		for (int k = 0; k < numPrimitives2D; k++) {
			for (int j = X; j <= Y; j++) {
				// advance one step
				pos2D[k][j] += vel2D[k][j];
				// if hit a wall, reverse direction
				if (pos2D[k][j] < min2D[j] || pos2D[k][j] > max2D[j])
					vel2D[k][j] = -vel2D[k][j];
			}
		}

		// update 3D postions
		for (int k = 0; k < numPrimitives3D; k++) {
			for (int j = 0; j <= Z; j++) {
				// advance one step
				pos3D[k][j] += vel3D[k][j];
				// if hit a wall, reverse direction
				// walls are sides of the unit cube
				if (pos3D[k][j] < 0. || pos3D[k][j] > 1.)
					vel3D[k][j] = -vel3D[k][j];
			}
		}
	}

}
