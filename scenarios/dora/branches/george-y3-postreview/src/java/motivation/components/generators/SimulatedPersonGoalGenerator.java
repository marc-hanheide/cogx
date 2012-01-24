/**
 * 
 */
package motivation.components.generators;

import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.util.ArrayList;

import javax.swing.JButton;

import VisionData.Person;
import cast.CASTException;
import cast.cdl.WorkingMemoryAddress;
import eu.cogx.planner.facade.ManualPlanningTaskComponent;
import execution.slice.person.PersonObservation;

/**
 * @author cogx
 * 
 */
public class SimulatedPersonGoalGenerator extends ManualPlanningTaskComponent {

	public static final int NUMBER_PERSON_OBSERVATIONS = 5;

	@Override
	protected void start() {
		super.start();
		JButton addHumanButton = new JButton("add human");
		getFrame().getJButtonPanel().add(addHumanButton);
		getFrame().pack();
		addHumanButton.addActionListener(new ActionListener() {

			@Override
			public void actionPerformed(ActionEvent e) {
				PersonObservation po = new PersonObservation(
						new ArrayList<Person>(), 1.0, 0, 0, 0, 0, 0,
						"commander");
				try {
					for (int i = 0; i < NUMBER_PERSON_OBSERVATIONS; i++)
						addToWorkingMemory(new WorkingMemoryAddress(
								newDataID(), "vision.sa"), po);
				} catch (CASTException e1) {
					logException(e1);
				}
			}
		});

	}

}
