package verbalisation;

import java.awt.BorderLayout;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.util.ArrayList;

import javax.swing.JButton; 
import javax.swing.JFrame;
import javax.swing.JLabel;
import javax.swing.JPanel;
import javax.swing.JScrollPane;
import javax.swing.JTable;
import javax.swing.JTextField;
import javax.swing.ListSelectionModel;
import javax.swing.event.ListSelectionEvent;
import javax.swing.event.ListSelectionListener;
import javax.swing.table.AbstractTableModel;
import javax.swing.table.DefaultTableModel;

import autogen.Planner.PlanningTask;

/** GUI to keep track of PlanningTasks and trigger verbalisation.
 * @author Graham Horn
*/
public class PlanVerbalisationControllerFrame extends JFrame
{
	{
		//Set Look & Feel
		try {
			javax.swing.UIManager.setLookAndFeel("com.sun.java.swing.plaf.gtk.GTKLookAndFeel");
		} catch(Exception e) {
			e.printStackTrace();
		}
	}

  private PlanVerbalisationController planVerbalisationController;
  private JTable tasksTable;
  private PlanningTaskTableModel tasksTableModel;
  
  private JPanel controlsPanel;
  private JButton verbaliseButton;
  private JTextField taskIdField;
  
  public PlanVerbalisationControllerFrame(PlanVerbalisationController plan_verbalisation_controller)
  {
    super("Plan Verbalisation Control");
    planVerbalisationController = plan_verbalisation_controller;
    getContentPane().setLayout(new BorderLayout());
    
    tasksTableModel = new PlanningTaskTableModel();
    tasksTable = new JTable(tasksTableModel);   
    tasksTable.setModel(tasksTableModel);
    tasksTable.getColumnModel().getColumn(0).setPreferredWidth(80);
    tasksTable.getColumnModel().getColumn(2).setPreferredWidth(150);
    tasksTable.getColumnModel().getColumn(0).setMaxWidth(90);
    tasksTable.getColumnModel().getColumn(2).setMaxWidth(180);
    tasksTable.getColumnModel().getColumn(0).setMinWidth(80);
    tasksTable.getColumnModel().getColumn(2).setMinWidth(150);
    tasksTable.setSelectionMode(ListSelectionModel.SINGLE_SELECTION);
    tasksTable.getSelectionModel().addListSelectionListener(new ListSelectionListener()
    {    
      public void valueChanged(ListSelectionEvent e)
      {
        if (tasksTable.getSelectedRow() >= 0)
        {
          taskIdField.setText(""+tasksTableModel.getTaskId(tasksTable.getSelectedRow()));
        }
      }
    });
    
    JScrollPane scroll_pane = new JScrollPane(tasksTable);
    tasksTable.setFillsViewportHeight(true);

    controlsPanel = new JPanel();
    taskIdField = new JTextField("      ");
    verbaliseButton = new JButton("Verbalise");
    controlsPanel.add(verbaliseButton);
    controlsPanel.add(new JLabel(" task ID = "));
    controlsPanel.add(taskIdField);
    verbaliseButton.addActionListener(new ActionListener()
    {    
      public void actionPerformed(ActionEvent action_event) 
      {
        try
        {
          int planning_task_id = Integer.parseInt(taskIdField.getText());
          
          boolean trigger_result = planVerbalisationController.triggerPlanVerbalisation(planning_task_id);
          
          if (trigger_result == false)
          {
            System.err.println("PlanVerbalisationController: Invalid task id");
          }
        }
        catch (NumberFormatException e)
        {
          // invalid content
        }
      }
    });
    getContentPane().add(scroll_pane, BorderLayout.CENTER);
    getContentPane().add(controlsPanel, BorderLayout.SOUTH);
  }
  
  public void addPlanningTask(PlanningTask new_planning_task)
  {
    tasksTableModel.updatePlanningTask(new_planning_task);
  }

  public void updatePlanningTask(PlanningTask planning_task)
  {
    tasksTableModel.updatePlanningTask(planning_task);
  }
  
  private class PlanningTaskTableModel extends AbstractTableModel
  {
    public static final int TASK_ID_COLUMN = 0;
    public static final int GOAL_COLUMN = 1;
    public static final int STATUS_COLUMN = 2;

    private String[] columnNames = { "Task ID", "Goal", "Execution Status"};
    private ArrayList<PlanningTask> planningTasks;
    
    public PlanningTaskTableModel() {
      planningTasks = new ArrayList<PlanningTask>();
    }

    public int getTaskId(int row)
    {
      return planningTasks.get(row).id;
    }

    public int getColumnCount() {
      return columnNames.length;
    }

    public int getRowCount() {
      return planningTasks.size();
    }

    public String getColumnName(int col) {
      return columnNames[col];
    }

    public Object getValueAt(int row, int col) {
      PlanningTask planning_task = planningTasks.get(row);
      switch (col)
      {
        case TASK_ID_COLUMN:
        {
          return planning_task.id;
        }
        case GOAL_COLUMN:
        {
          return PlanVerbalisationController.goalToString(planning_task.goals[0]);
        }
        case STATUS_COLUMN:
        {
          return PlanVerbalisationController.planningTaskExecutionStatusToString(planning_task);
        }
      }
      return null;
    }
  
    
    public void updatePlanningTask(PlanningTask planning_task)
    {
      boolean found_id = false;

      for (PlanningTask stored_planning_task : planningTasks)
      {
        if (stored_planning_task.id == planning_task.id)
        {
          stored_planning_task.executionStatus = planning_task.executionStatus;
          found_id = true;
        }
      }
      
      if (found_id == false)
      {
        planningTasks.add(planning_task);
      }
      else
      {
        
      }
      fireTableDataChanged();
    }
  }
  
}
