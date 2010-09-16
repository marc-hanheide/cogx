package vision.components;

import java.util.HashMap;
import java.util.Map;

import javax.swing.JOptionPane;

import vision.VisionUtils;
import VisionData.DetectionCommand;
import VisionData.VisualObject;
import cast.CASTException;
import cast.architecture.ChangeFilterFactory;
import cast.architecture.ManagedComponent;
import cast.architecture.WorkingMemoryChangeReceiver;
import cast.cdl.WorkingMemoryChange;
import cast.cdl.WorkingMemoryOperation;

/**
 * A fake object detector.
 * 
 * Configure options: --confidence-X=label1,label2,...,labelN: return confidence
 * X for each label. e.g. --confidence-0.6=tea,biscuits
 * 
 * --default-confidence -> the value to give all detections which are not
 * recognised
 * 
 * 
 * @author nah
 * 
 */
public class ObjectDetectorFAKE extends ManagedComponent implements
		WorkingMemoryChangeReceiver {

	private static final String DEFAULT_CONFIDENCE_PREFIX = "--default-confidence";

	private static final String LABEL_CONFIG_PREFIX = "--confidence-";
	private static final int LABEL_CONFIG_PREFIX_LENGTH = LABEL_CONFIG_PREFIX
			.length();

	private boolean m_gui = false;	
	private final Map<String, Double> m_label2confidence;
	private double m_defaultConfidence;

	public ObjectDetectorFAKE() {
		m_label2confidence = new HashMap<String, Double>();
		m_defaultConfidence = 0d;
	}

	@Override
	protected void start() {
		addChangeFilter(ChangeFilterFactory.createTypeFilter(
				DetectionCommand.class, WorkingMemoryOperation.ADD), this);

	}

	@Override
	public void workingMemoryChanged(WorkingMemoryChange _arg0)
			throws CASTException {

		// load command
		DetectionCommand dc = getMemoryEntry(_arg0.address,
				DetectionCommand.class);

		// NEW CODE
		// ask the user for a selection (added: hz, 2009-10-26)
		String objectLabel = "";
		
		if (m_gui) {
			objectLabel = (String) JOptionPane.showInputDialog(
				null, null, "Select an object to recognize", 
				JOptionPane.PLAIN_MESSAGE, null, dc.labels, null);
		}
		
//		because vision is never this quick...
		sleepComponent(2000);

		for (String label : dc.labels) {
			// for the time being just fail
			VisualObject obj = VisionUtils.newVisualObject();
		
			if (m_gui) {
				if (label.equals(objectLabel)) {
					obj.detectionConfidence = getConfidenceForLabel(label);
				} else {
					obj.detectionConfidence = m_defaultConfidence;
				}
			} else {
				obj.detectionConfidence = getConfidenceForLabel(label);
			}
			obj.identLabels=new String[1];
			obj.identLabels[0] = label;
			addToWorkingMemory(newDataID(), obj);
		}

	}

	@Override
	protected void configure(Map<String, String> _arg0) {

		for (String key : _arg0.keySet()) {
			if (key.startsWith(LABEL_CONFIG_PREFIX)) {
				// parse out confidence
				String confString = key.substring(LABEL_CONFIG_PREFIX_LENGTH);
				try {
					double labelConfidence = Double.parseDouble(confString);
					String labelsJoined = _arg0.get(key);
					String[] labels = labelsJoined.split(",");
					for (String label : labels) {
						m_label2confidence.put(label, labelConfidence);
						log("fakey fakey: " + label + " -> " + labelConfidence);
					}
				} catch (NumberFormatException e) {
					println("Unable to parse confidence string: " + confString);
				}
			}
		}

		String defaultString = _arg0.get(DEFAULT_CONFIDENCE_PREFIX);
		if (defaultString != null) {
			m_defaultConfidence = Double.parseDouble(defaultString);
		}
		log("default confidence: " + m_defaultConfidence);
		
		String guiFlag = _arg0.get("--gui");
		if (guiFlag != null) {
			if (!guiFlag.toLowerCase().equals("false")) {
				m_gui = true;
			}
		}
		log("m_gui: " + (m_gui ? "true" : "false"));		
	}

	/**
	 * 
	 * over 0.5 is a detection for AVS
	 * 
	 * @param _label
	 * @return
	 */
	protected double getConfidenceForLabel(String _label) {
		Double conf = m_label2confidence.get(_label);
		if(conf != null) {
			return conf;
		}
		else {
			return m_defaultConfidence;
		}
	}


	

}
