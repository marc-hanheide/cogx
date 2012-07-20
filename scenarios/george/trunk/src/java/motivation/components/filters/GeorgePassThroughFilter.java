/**
 * 
 */
package motivation.components.filters;

import java.awt.Dimension;
import java.util.Dictionary;
import java.util.HashMap;
import java.util.Hashtable;
import java.util.Map;

import javax.swing.JComponent;
import javax.swing.JLabel;
import javax.swing.JSlider;

import motivation.slice.AnalyzeProtoObjectMotive;
import motivation.slice.LearnObjectFeatureMotive;
import motivation.slice.LookAtViewConeMotive;
import motivation.slice.Motive;
import motivation.slice.MotivePriority;
import motivation.slice.MotiveStatus;
import motivation.slice.RobotNonSituatedMotive;
import motivation.slice.TutorInitiativeMotive;
import motivation.util.WMMotiveView;
import cast.CASTException;
import cast.cdl.WorkingMemoryChange;
import cast.cdl.WorkingMemoryPermissions;

/**
 * @author marc
 * 
 */
public class GeorgePassThroughFilter extends AbstractManualSelectFilter {

	private DelayFilterDisplayClient m_display;

	protected class OnOffFilterPanel extends FilterPanel {

		/**
		 * 
		 */
		private static final long serialVersionUID = 1L;

		public OnOffFilterPanel(Class<? extends Motive> type) {
			super(type);
			this.setPreferredSize(new Dimension(200, 50));
		}

		@Override
		public MotivePriority getPriority() {
			if (getSlider().getValue() == 0) {
				return MotivePriority.UNSURFACE;
			} else {
				return MotivePriority.NORMAL;
			}
		}

		@Override
		public JSlider getSlider() {
			if (jSlider == null) {
				jSlider = new JSlider(JSlider.HORIZONTAL, 0, 1, 1);
				Dictionary<Integer, JComponent> labels;
				labels = new Hashtable<Integer, JComponent>();
				{
					JLabel l = new JLabel("Off");
					l.setFont(l.getFont().deriveFont(9f));
					labels.put(0, l);
				}
				{
					JLabel l = new JLabel("On");
					l.setFont(l.getFont().deriveFont(9f));
					labels.put(1, l);
				}

				jSlider.setLabelTable(labels);
				jSlider.setPaintTicks(true);
				jSlider.setPaintLabels(true);
				jSlider.setSnapToTicks(true);
			}
			return jSlider;

		}

	}

	public static void main(String[] args) {
		AbstractManualSelectFilter o = new GeorgePassThroughFilter();
		o.start();
	}

	@Override
	public void configure(Map<String, String> _config) {
		super.configure(_config);
		m_display = DelayFilterDisplayClient.getClient(_config);
	}

	@Override
	public void start() {
		super.start();
		m_display.connect(component);
	}

	/**
	 * @throws ClassNotFoundException
	 * 
	 */
	protected FilterPanel getFilterPanel(Class<? extends Motive> c) {
		if (panels.get(c) == null) {
			OnOffFilterPanel p = new OnOffFilterPanel(c);
			panels.put(c, p);
			return p;
		} else {
			return panels.get(c);
		}
	}

	@Override
	protected void onUnsurfaceAll() {
		WMMotiveView motives = component.getMotives();

		for (Motive mtv : motives.values()) {
			if (mtv.status == MotiveStatus.ACTIVE) {
				try {
					component.lockEntry(mtv.thisEntry,
							WorkingMemoryPermissions.LOCKEDO);
					Motive motive = component.getMemoryEntry(mtv.thisEntry,
							Motive.class);
					motive.status = MotiveStatus.COMPLETED;
					component.overwriteWorkingMemory(motive.thisEntry, motive);
				} catch (Exception e) {
					component.logException(e);
				} finally {
					try {
						component.unlockEntry(mtv.thisEntry);
					} catch (CASTException e) {
						component.logException(
								"CASTException in motive filtering ", e);
					}
				}
			}
		}

	}

	@Override
	public MotivePriority checkMotive(Motive motive, WorkingMemoryChange wmc) {
		MotivePriority priority = super.checkMotive(motive, wmc);

		HashMap<Class<? extends Motive>, Integer> values = new HashMap<Class<? extends Motive>, Integer>();
		for (FilterPanel p : panels.values()) {
			values.put(p.getType(), p.getPriority().ordinal());
		}

		m_display.setPassThroughState(values, component.getMotives());

		// component.println("manual set: " + motive.getClass() + " " +
		// priority);

		return priority;
	}

	@Override
	protected void registerTypes() {
		addDefault("Analyse Objects", AnalyzeProtoObjectMotive.class,
				MotivePriority.NORMAL);
		addDefault("Tutor-Driven Motives", TutorInitiativeMotive.class,
				MotivePriority.NORMAL);
		addDefault("Robot-Driven Motives", LearnObjectFeatureMotive.class,
				MotivePriority.NORMAL);
		addDefault("Look Around", LookAtViewConeMotive.class,
				MotivePriority.NORMAL);
		addDefault("Non-Situated Motives", RobotNonSituatedMotive.class,
				MotivePriority.NORMAL);

	}

}
