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
public class ReaderViewAnnotator extends ComponentViewAnnotator {

    private Label m_changesTypeLabel;
    private Label m_changesValueLabel;
    private Label m_readsTypeLabel;
    private Label m_readsValueLabel;

    private UpdateRunnable m_changesValueRunnable;
    private UpdateRunnable m_readsValueRunnable;
    
    /**
     * @param _view
     * @param _monitor
     */
    public ReaderViewAnnotator(Composite _view,
            ComponentMonitor _monitor) {
        super(_monitor,_view.getDisplay());
        
        
        String changesText = "queued changes/total filtered/total received";
        m_changesTypeLabel = new Label(_view, SWT.NONE);
        m_changesTypeLabel.setText("changes: ");
        m_changesTypeLabel.setToolTipText(changesText);
        m_changesValueLabel = new Label(_view, SWT.NONE);
        m_changesValueLabel.setText("  0/  0/  0");
        m_changesValueLabel.setToolTipText(changesText);
        m_changesValueRunnable = new UpdateRunnable(m_changesValueLabel);
        
        String readsText = "wm. reads";
        m_readsTypeLabel = new Label(_view, SWT.NONE);
        m_readsTypeLabel.setText("reads: ");
        m_readsTypeLabel.setToolTipText(readsText);
        m_readsValueLabel = new Label(_view, SWT.NONE);
        m_readsValueLabel.setText("  0");
        m_readsValueLabel.setToolTipText(readsText);
        m_readsValueRunnable = new UpdateRunnable(m_readsValueLabel);
    }

    private String getChangesString() {
        StringBuffer sb = new StringBuffer();
        sb.append(m_monitor.getChangeQueueLength());
        sb.append("/");
        sb.append(m_monitor.getTotalFilteredEvents());
        sb.append("/");
        sb.append(m_monitor.getTotalEventsReceived());
        return sb.toString();
    }
    
    private String getReadsString() {
        StringBuffer sb = new StringBuffer();
        sb.append(m_monitor.getTotalReads());
        return sb.toString();
    }

    
    /*
     * (non-Javadoc)
     * 
     * @see caat.ui.architecture.views.ComponentViewAnnotator#update()
     */
    @Override
    protected void update() {
        m_changesValueRunnable.setText(getChangesString());
        m_display.asyncExec(m_changesValueRunnable);
        m_readsValueRunnable.setText(getReadsString());
        m_display.asyncExec(m_readsValueRunnable);
//        m_readsValueLabel.pack(true);
    }


}
