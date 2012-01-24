package vision.components;

import java.util.HashMap;
import java.util.Map;

import org.apache.log4j.Logger;

import VisionData.VisualObject;
import cast.CASTException;
import cast.architecture.ChangeFilterFactory;
import cast.architecture.ManagedComponent;
import cast.architecture.WorkingMemoryChangeReceiver;
import cast.cdl.WorkingMemoryChange;
import cast.cdl.WorkingMemoryOperation;

/**
 * 
 * @author Graham Horn
 *
 */
public class ObjectDetectionDriver extends ManagedComponent implements
    WorkingMemoryChangeReceiver {

  private static final String GUI_KEY = "--gui";
  private static final String LABELS_KEY = "--labels";
  
  private String[] labels;
  private boolean useGui = false;
  private Map<String, Boolean> everDetected;
  private Map<String, Boolean> currentlyDetected;
  private ObjectDetectionDriverGUI gui;

  private Logger logger = Logger.getLogger(this.getClass());

  public String[] getLabels() {
    return labels;
  }
  
  @Override
  public void workingMemoryChanged(WorkingMemoryChange wmc)
      throws CASTException {
    VisualObject visual_object = getMemoryEntry(wmc.address, VisualObject.class);
    
    if (visual_object.identDistrib[0] >=0.5)
    {
      logger.debug("Detected " + visual_object.identLabels[0]);
      everDetected.put(visual_object.identLabels[0], Boolean.TRUE);
      currentlyDetected.put(visual_object.identLabels[0], Boolean.TRUE);
    }
    else
    {
      logger.debug("Did not detect " + visual_object.identLabels[0]);
      currentlyDetected.put(visual_object.identLabels[0], Boolean.FALSE);
    }
    
   if (useGui)
   {
     gui.showDetections(everDetected, currentlyDetected);
   }
    
  }

  @Override
  protected void configure(Map<String, String> _config) {
    
    everDetected = new HashMap<String, Boolean>();
    currentlyDetected = new HashMap<String, Boolean>();
    
    if (_config.containsKey(LABELS_KEY)) {
      labels = _config.get(LABELS_KEY).split(" ");     
    }
    else {
      labels = new String[0];
    }
    
    for (int i = 0; i < labels.length; i++) {
      everDetected.put(labels[i], Boolean.FALSE);
      currentlyDetected.put(labels[i], Boolean.FALSE);
    }
    
    if (_config.containsKey(GUI_KEY)) {
      useGui = true;
    }
    
  }

  @Override
  protected void start() {
      addChangeFilter(ChangeFilterFactory.createLocalTypeFilter(VisualObject.class,
          WorkingMemoryOperation.ADD), this);
      addChangeFilter(ChangeFilterFactory.createLocalTypeFilter(VisualObject.class,
          WorkingMemoryOperation.OVERWRITE), this);      
 
    if (useGui){
      gui = new ObjectDetectionDriverGUI(this);
    }
  }

}
