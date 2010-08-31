/** _
 *  part of the ELM system.
 *  @author Dennis Stachowicz
 */

package elm.tests;

public class EventSimulatorTest {

	public static void main(String[] args) {
		EventSimulator es = new EventSimulator();

		System.out.println("Making ten moves:");
		for (int i = 0; i < 10; i++) {
			es.move();
			es.printPositionAndTime();
		}
		
	}	
}
