package motivation.components.filters;

import java.awt.BorderLayout;
import java.awt.Dimension;
import java.awt.GridLayout;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.util.Dictionary;
import java.util.HashMap;
import java.util.Hashtable;
import java.util.LinkedHashMap;
import java.util.Map;
import java.util.Map.Entry;

import javax.swing.JButton;
import javax.swing.JComponent;
import javax.swing.JFrame;
import javax.swing.JLabel;
import javax.swing.JPanel;
import javax.swing.JSeparator;
import javax.swing.JSlider;

import motivation.slice.ExploreMotive;
import motivation.slice.Motive;
import motivation.slice.MotivePriority;
import motivation.slice.PatrolMotive;
import motivation.slice.TutorInitiativeLearningMotive;
import cast.CASTException;
import cast.cdl.WorkingMemoryChange;

public class AbstractManualSelectFilter implements MotiveFilter {

	Map<Class<? extends Motive>, AbstractManualSelectFilter.FilterPanel> panels = new LinkedHashMap<Class<? extends Motive>, AbstractManualSelectFilter.FilterPanel>();

	Map<String, Map<Class<? extends Motive>, MotivePriority>> defaults = new LinkedHashMap<String, Map<Class<? extends Motive>, MotivePriority>>();

	/**
	 * @throws ClassNotFoundException
	 * 
	 */
	protected FilterPanel getFilterPanel(Class<? extends Motive> c) {
		if (panels.get(c) == null) {
			FilterPanel p = new FilterPanel(c);
			panels.put(c, p);
			return p;
		} else {
			return panels.get(c);
		}

	}

	protected void addDefault(String name, Class<? extends Motive> c,
			MotivePriority prio) {
		Map<Class<? extends Motive>, MotivePriority> d = defaults.get(name);
		if (d == null) {
			d = new HashMap<Class<? extends Motive>, MotivePriority>();
			defaults.put(name, d);
		}
		d.put(c, prio);
		if (panels.get(c) == null)
			getFilterPanel(c);
	}

	protected FilterPanel getFilterPanel(String type) throws ClassNotFoundException {
		@SuppressWarnings("unchecked")
		Class<? extends Motive> c = (Class<? extends Motive>) Class
				.forName(type);
		return getFilterPanel(c);
	}

	public AbstractManualSelectFilter() {
	}

	protected class FilterPanel extends JPanel {
		/**
		 * 
		 */
		private static final long serialVersionUID = 1L;
		private JSlider jSlider = null;
		private Class<? extends Motive> type;

		public FilterPanel(Class<? extends Motive> type) {
			super();
			this.type = type;
			initialize();
		}

		private void initialize() {
			this.setLayout(new BorderLayout());
			this.add(new JLabel(type.getSimpleName()), BorderLayout.NORTH);
			this.add(getSlider(), BorderLayout.CENTER);
			this.setPreferredSize(new Dimension(500, 80));
			this.add(new JSeparator(),BorderLayout.SOUTH);
		}

		public Class<? extends Motive> getType() {
			return type;
		}

		public JSlider getSlider() {
			if (jSlider == null) {
				jSlider = new JSlider(JSlider.HORIZONTAL,
						MotivePriority.UNSURFACE.ordinal(),
						MotivePriority.HIGH.ordinal(),
						MotivePriority.UNSURFACE.ordinal());
				Dictionary<Integer, JComponent> labels;
				labels = new Hashtable<Integer, JComponent>();
				labels.put(MotivePriority.UNSURFACE.ordinal(), new JLabel(
						MotivePriority.UNSURFACE.name()));
				labels.put(MotivePriority.LOW.ordinal(), new JLabel(
						MotivePriority.LOW.name()));
				labels.put(MotivePriority.NORMAL.ordinal(), new JLabel(
						MotivePriority.NORMAL.name()));
				labels.put(MotivePriority.HIGH.ordinal(), new JLabel(
						MotivePriority.HIGH.name()));
				jSlider.setLabelTable(labels);
				jSlider.setPaintTicks(true);
				jSlider.setPaintLabels(true);
				jSlider.setSnapToTicks(true);
			}
			return jSlider;

		}

		public MotivePriority getPriority() {
			return MotivePriority.values()[getSlider().getValue()];
		}

	}

	private MotiveFilterManager component; // @jve:decl-index=0:

	private JFrame jFrame = null; // @jve:decl-index=0:visual-constraint="1,21"
	private JPanel jContentPane = null;
	private JButton jButtonUpdate = null; // @jve:decl-index=0:visual-constraint="325,118"

	private JPanel jMotivesPanel = null;

	public MotivePriority checkMotive(Motive motive, WorkingMemoryChange wmc) {
		for (FilterPanel p : panels.values()) {
			if (motive.getClass().equals(p.getType()))
				return p.getPriority();
		}
		return MotivePriority.UNSURFACE;
	}

	/**
	 * This method initializes jFrame
	 * 
	 * @return javax.swing.JFrame
	 */
	private JFrame getJFrame() {
		if (jFrame == null) {
			jFrame = new JFrame();
			jFrame.setTitle("Motivation::ManualSelectFilter");
			jFrame.setContentPane(getJContentPane());
		}
		return jFrame;
	}

	/**
	 * This method initializes jContentPane
	 * 
	 * @return javax.swing.JPanel
	 */
	private JPanel getJContentPane() {
		if (jContentPane == null) {
			jContentPane = new JPanel();
			jContentPane.setLayout(new GridLayout(0, 1));
			jContentPane.add(getJMotivesPanel());
			jMotivesPanel.add(getJButtonUpdate());
		}
		return jContentPane;
	}

	/**
	 * This method initializes jButton
	 * 
	 * @return javax.swing.JButton
	 */
	private JButton getJButtonUpdate() {
		if (jButtonUpdate == null) {
			jButtonUpdate = new JButton();
			jButtonUpdate.setText("recheck all");
			jButtonUpdate.addActionListener(new ActionListener() {

				@Override
				public void actionPerformed(ActionEvent e) {
					if (component != null) {
						try {
							component.println("update everything");
							component.checkAll();
						} catch (CASTException e1) {
							component
									.println("unexpected exception in checkAll: ");
							e1.printStackTrace();
						}
					}
				}
			});
		}
		return jButtonUpdate;
	}

	public void setManager(MotiveFilterManager motiveFilterManager) {
		component = motiveFilterManager;
	}

	/**
	 * This method initializes jMotivesPanel
	 * 
	 * @return javax.swing.JPanel
	 */
	private JPanel getJMotivesPanel() {
		if (jMotivesPanel == null) {
			GridLayout gridLayout = new GridLayout(0, 1);
			// gridLayout.setColumns(1);
			jMotivesPanel = new JPanel();
			jMotivesPanel.setLayout(gridLayout);
			for (FilterPanel p : panels.values()) {
				jMotivesPanel.add(p);
				
			}
			jMotivesPanel.add(getPresetPanel());

		}
		return jMotivesPanel;
	}

	private JPanel getPresetPanel() {
		JPanel presetPanel = new JPanel();
		presetPanel.add(new JLabel("presets:"));

		JButton nullButton = new JButton("unsurface all");
		nullButton.addActionListener(new ActionListener() {
			@Override
			public void actionPerformed(ActionEvent e) {
				try {
					for (FilterPanel p : panels.values()) {
						p.getSlider().setValue(0);
					}
					if (component != null)
						component.checkAll();
				} catch (CASTException e1) {
					component.logException(e1);
				}

			}
		});
		presetPanel.add(nullButton);

		for (final Entry<String, Map<Class<? extends Motive>, MotivePriority>> def : defaults
				.entrySet()) {
			JButton button = new JButton(def.getKey());
			button.addActionListener(new ActionListener() {
				@Override
				public void actionPerformed(ActionEvent e) {
					try {
						for (FilterPanel p : panels.values()) {
							MotivePriority prio = def.getValue().get(
									p.getType());
							if (prio != null)
								p.getSlider().setValue(prio.ordinal());
						}
						if (component != null)
							component.checkAll();
					} catch (CASTException e1) {
						component.logException(e1);
					}

				}
			});
			presetPanel.add(button);

		}

		return presetPanel;
	}

	@Override
	public void start() {
		registerTypes();
		getJFrame().setVisible(true);
		getJFrame().pack();
		// getJFrame().setSize(800, 600);
	}

	protected void registerTypes() {
		addDefault("test Default", ExploreMotive.class, MotivePriority.HIGH);
		addDefault("test 2", PatrolMotive.class, MotivePriority.NORMAL);
		addDefault("test 2", ExploreMotive.class, MotivePriority.NORMAL);
		getFilterPanel(ExploreMotive.class);
		getFilterPanel(PatrolMotive.class);
		getFilterPanel(TutorInitiativeLearningMotive.class);
	}

	@Override
	public void configure(Map<String, String> arg0) {
		// TODO Auto-generated method stub

	}

	public static void main(String[] args) {
		AbstractManualSelectFilter o = new AbstractManualSelectFilter();
		o.start();
	}

} // @jve:decl-index=0:visual-constraint="583,36"
