package demos.robotville;

import java.awt.EventQueue;
import java.awt.GridLayout;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;

import javax.swing.JButton;
import javax.swing.JFrame;
import javax.swing.JPanel;
import javax.swing.JTextField;
import javax.swing.border.EmptyBorder;

public class RobotVilleJFrame extends JFrame {

	private JPanel contentPane;
	private JTextField txtCurrently;
	protected RobotVilleComponent component;

	/**
	 * Launch the application.
	 */
	public static void main(String[] args) {
		EventQueue.invokeLater(new Runnable() {
			public void run() {
				try {
					RobotVilleJFrame frame = new RobotVilleJFrame();
					frame.setVisible(true);
				} catch (Exception e) {
					e.printStackTrace();
				}
			}
		});
	}

	/**
	 * Create the frame.
	 */
	public RobotVilleJFrame() {
		createJFrame();
	}

	/**
	 * Create the frame.
	 */
	public RobotVilleJFrame(RobotVilleComponent component) {
		this.component = component;
		createJFrame();
	}

	/**
	 * 
	 */
	private void createJFrame() {
		setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
		setBounds(100, 100, 450, 300);
		contentPane = new JPanel();
		contentPane.setBorder(new EmptyBorder(5, 5, 5, 5));
		setContentPane(contentPane);
		contentPane.setLayout(new GridLayout(0, 1, 0, 0));

		JButton btnExplore = new JButton("Explore");
		btnExplore.addActionListener(new ActionListener() {
			public void actionPerformed(ActionEvent arg0) {
				component.stopRunningTasks();
				component
						.startPlannedAction("(forall (?p - place) (= (placestatus ?p) trueplace))");
				setStatus("started explore task");
			}

		});
		contentPane.add(btnExplore);

		JButton btnFindPerson = new JButton("Find Person");
		btnFindPerson.addActionListener(new ActionListener() {
			public void actionPerformed(ActionEvent arg0) {
				component.stopRunningTasks();
				component
						.startPlannedAction("(exists (?p - person) (kval ROBOT (is-in ?p)))");
				setStatus("started explore task");
			}
		});
		contentPane.add(btnFindPerson);

		JButton btnPatrol = new JButton("Patrol");
		btnPatrol.addActionListener(new ActionListener() {
			public void actionPerformed(ActionEvent arg0) {
				component.startPatrolBehaviour();
			}
		});
		contentPane.add(btnPatrol);

		JButton btnIdle = new JButton("Idle");
		btnIdle.addActionListener(new ActionListener() {
			public void actionPerformed(ActionEvent arg0) {
				component.startIdleBehaviour();
			}
		});
		contentPane.add(btnIdle);
		
		JButton btnGoHome = new JButton("Go Home");
		btnGoHome.addActionListener(new ActionListener() {
			public void actionPerformed(ActionEvent arg0) {
				component.goHome();
			}
		});
		contentPane.add(btnGoHome);

		txtCurrently = new JTextField();
		txtCurrently.setText("Current status");
		contentPane.add(txtCurrently);
		txtCurrently.setColumns(10);

		JButton btnStop = new JButton("Stop");
		btnStop.addActionListener(new ActionListener() {
			public void actionPerformed(ActionEvent arg0) {
				component.stopRunningTasks();
			}
		});
		contentPane.add(btnStop);
	}

	protected void setStatus(String string) {
		txtCurrently.setText(string);

	}

}
