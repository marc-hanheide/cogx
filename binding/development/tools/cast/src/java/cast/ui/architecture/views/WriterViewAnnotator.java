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
public class WriterViewAnnotator extends ComponentViewAnnotator {

    private Label m_writesTypeLabel;
    private Label m_writesValueLabel;
    private UpdateRunnable m_writesValueRunnable;
    /**
     * @param _view
     * @param _monitor
     */
    public WriterViewAnnotator(Composite _view,
            ComponentMonitor _monitor) {
        super(_monitor,_view.getDisplay());
        
        String writesText = "adds/overwrites/deletes";
        m_writesTypeLabel = new Label(_view, SWT.NONE);
        m_writesTypeLabel.setText("writes: ");
        m_writesTypeLabel.setToolTipText(writesText);
        m_writesValueLabel = new Label(_view, SWT.NONE);
        m_writesValueLabel.setText("  0/  0/  0");
        m_writesValueLabel.setToolTipText(writesText);
        m_writesValueRunnable = new UpdateRunnable(m_writesValueLabel);
       
        
    }

    private String getWritesString() {
        StringBuffer sb = new StringBuffer();
        sb.append(m_monitor.getTotalAdds());
        sb.append("/");
        sb.append(m_monitor.getTotalOverwrites());
        sb.append("/");
        sb.append(m_monitor.getTotalDeletes());
        return sb.toString();
    }
    

    
    /*
     * (non-Javadoc)
     * 
     * @see caat.ui.architecture.views.ComponentViewAnnotator#update()
     */
    @Override
    protected void update() {
        m_writesValueRunnable.setText(getWritesString());
        m_display.asyncExec(m_writesValueRunnable);
    }


}
