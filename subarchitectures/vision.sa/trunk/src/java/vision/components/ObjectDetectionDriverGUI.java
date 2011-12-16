package vision.components;

import java.awt.BorderLayout;
import java.awt.Container;
import java.awt.Dimension;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.util.Iterator;
import java.util.Map;

import javax.swing.JButton;
import javax.swing.JFrame;
import javax.swing.JLabel;
import javax.swing.JPanel;
import javax.swing.JTextArea;

import org.apache.log4j.Logger;

import VisionData.DetectionCommand;
import cast.AlreadyExistsOnWMException;

/**
 * 
 * @author Graham Horn
 *
 */
public class ObjectDetectionDriverGUI extends JPanel implements ActionListener{

  private static final long serialVersionUID = 2344450355031974257L;
  
  private Logger logger = Logger.getLogger(this.getClass());
  
  private ObjectDetectionDriver server;
  
  private JTextArea detectionResults;
  
  private JButton triggerDetection;
  
  public ObjectDetectionDriverGUI(ObjectDetectionDriver _server){
    server = _server;
    guiSetup();
  }
  
  
  /**
   * constructor to show the GUI without any functionality
   */
  public ObjectDetectionDriverGUI()
  {
    guiSetup();
  }
  
  private void guiSetup()
  {
    JFrame gui = new JFrame("Object Detection Driver GUI");
    gui.setDefaultCloseOperation(JFrame.DISPOSE_ON_CLOSE);
    gui.setLocation(100, 200);
    gui.setPreferredSize(new Dimension(300, 200));
    
    Container pane = gui.getContentPane();
    pane.setLayout(new BorderLayout());
    pane.add(new JLabel("Detected objects:"), BorderLayout.NORTH);
    detectionResults = new JTextArea();
    pane.add(detectionResults, BorderLayout.CENTER);
    triggerDetection = new JButton("Trigger detection");
    triggerDetection.addActionListener(this);
    pane.add(triggerDetection, BorderLayout.SOUTH);
    
    gui.pack();
    gui.setVisible(true);
  }
  
  public synchronized void showDetections(Map<String, Boolean> detection_results)
  {
    StringBuilder txt = new StringBuilder();
    for (Iterator<String> iterator = detection_results.keySet().iterator(); iterator.hasNext();) {
      String label = (String) iterator.next();
      txt.append(label).append(" - ").append(detection_results.get(label)).append("\n");
    }
    detectionResults.setText(txt.toString());
  }
  
  @Override
  public void actionPerformed(ActionEvent e) {
    DetectionCommand cmd = new DetectionCommand();
    cmd.labels = server.getLabels();
    String id = server.newDataID();
    
    try {
      logger.debug("Requesting detections");
      server.addToWorkingMemory(id, cmd);
     
    } catch (AlreadyExistsOnWMException e1) {
      logger.error(e1);
    }

  }

 
  /**
   * main method to show the GUI without any functionality
   * 
   * @param args
   */
  public static void main(String[] args) {
    new ObjectDetectionDriverGUI();
  }
}
