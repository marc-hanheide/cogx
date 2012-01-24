package castutils.experimentation;

import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.util.Map;

import javax.swing.JButton;
import javax.swing.JFrame;

import VisionData.DetectionCommand;
import VisionData.VisualObject;
import cast.AlreadyExistsOnWMException;
import cast.CASTException;
import cast.DoesNotExistOnWMException;
import cast.SubarchitectureComponentException;
import cast.UnknownSubarchitectureException;
import cast.architecture.ChangeFilterFactory;
import cast.architecture.ManagedComponent;
import cast.cdl.WorkingMemoryAddress;
import cast.cdl.WorkingMemoryChange;
import cast.cdl.WorkingMemoryOperation;
import castutils.castextensions.WMEventQueue;

public class ManualObservationModelEstimatorComponent extends ManagedComponent {

	private static final double MAGIC_CONFIDENCE_THRESHOLD = 0.08;
	private static final String DEFAULT_LABEL = "cerealbox";
	private static final String LABEL_CONFIG = "--label";

	@Override
	protected void configure(Map<String, String> config) {
		if (config.containsKey(LABEL_CONFIG)) {
			objectLabels[0] = config.get(LABEL_CONFIG);
		} else {
			objectLabels[0] = DEFAULT_LABEL;
		}
	}

	private String[] objectLabels = new String[1];
	private int numberLooks = 0;
	private int numberDetected = 0;

	@Override
	protected void start() {
		JFrame jFrame = new JFrame(
				ManualObservationModelEstimatorComponent.class.getSimpleName());
		final JButton button = new JButton("recognize");
		button.addActionListener(new ActionListener() {

			@Override
			public void actionPerformed(ActionEvent arg0) {
				recognize();
				println("Current observation model " + ((double) numberDetected)
						/ ((double) numberLooks));

			}
		});
		jFrame.add(button);
		jFrame.pack();
		jFrame.setVisible(true);

	}

	protected boolean recognize() {
		try {
			VisualObject nc = executeDetection();
			numberLooks++;
			if (nc.identDistrib[0] > MAGIC_CONFIDENCE_THRESHOLD) {
				numberDetected++;
				println("actually, I saw the object, now " + numberDetected
						+ " out of " + numberLooks);
				return true;

			}
		} catch (CASTException e) {
			logException(e);
		} catch (InterruptedException e) {
			logException(e);
		}
		println("I did not saw the object, now " + numberDetected
				+ " out of " + numberLooks);
		return false;
	}

	private VisualObject executeDetection() throws AlreadyExistsOnWMException,
			InterruptedException, DoesNotExistOnWMException,
			UnknownSubarchitectureException, SubarchitectureComponentException {
		WMEventQueue queue = new WMEventQueue();
		addChangeFilter(ChangeFilterFactory.createGlobalTypeFilter(
				VisualObject.class, WorkingMemoryOperation.ADD), queue);
		addChangeFilter(ChangeFilterFactory.createGlobalTypeFilter(
				VisualObject.class, WorkingMemoryOperation.OVERWRITE), queue);
		addToWorkingMemory(new WorkingMemoryAddress(newDataID(), "vision.sa"),
				new DetectionCommand(objectLabels));
		WorkingMemoryChange ev = queue.take();
		VisualObject nc = getMemoryEntry(ev.address, VisualObject.class);
		removeChangeFilter(queue);

		return nc;
	}

}
