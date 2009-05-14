/*
 * CAST - The CoSy Architecture Schema Toolkit
 *
 * Copyright (C) 2006-2007 Nick Hawes
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public License
 * as published by the Free Software Foundation; either version 2.1 of
 * the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
 * 02110-1301 USA
 *
 */

/**
 * 
 */
package cast.ui.architecture.views;

import org.eclipse.swt.SWT;
import org.eclipse.swt.widgets.Composite;
import org.eclipse.swt.widgets.Label;

import cast.ui.architecture.ComponentMonitor;

/**
 * @author nah
 */
public class ManagedViewAnnotator extends ComponentViewAnnotator {

    private Label m_tasksTypeLabel;
    private Label m_tasksValueLabel;
    private UpdateRunnable m_tasksValueRunnable;
    
    /**
     * @param _view
     * @param _monitor
     */
    public ManagedViewAnnotator(Composite _view,
            ComponentMonitor _monitor) {
   
        super(_monitor,_view.getDisplay());
        
        String tasksText = "proposed/accepted/completed";
        m_tasksTypeLabel = new Label(_view, SWT.NONE);
        m_tasksTypeLabel.setText("tasks: ");
        m_tasksTypeLabel.setToolTipText(tasksText);
        m_tasksValueLabel = new Label(_view, SWT.NONE);
        m_tasksValueLabel.setText("  0/  0/  0");
        m_tasksValueLabel.setToolTipText(tasksText);
        m_tasksValueRunnable = new UpdateRunnable(m_tasksValueLabel);
        
    }

    private String getTasksString() {
        StringBuffer sb = new StringBuffer();
        sb.append(m_monitor.getTotalProposals());
        sb.append("/");
        sb.append(m_monitor.getTotalStarts());
        sb.append("/");
        sb.append(m_monitor.getTotalEnds());
        return sb.toString();
    }
    

    
    /*
     * (non-Javadoc)
     * 
     * @see caat.ui.architecture.views.ComponentViewAnnotator#update()
     */
    @Override
    protected void update() {
        
        m_tasksValueRunnable.setText(getTasksString());
        m_display.asyncExec(m_tasksValueRunnable);
    }


}
