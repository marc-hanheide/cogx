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

import java.util.ArrayList;

import org.eclipse.swt.SWT;
import org.eclipse.swt.layout.GridData;
import org.eclipse.swt.layout.GridLayout;
import org.eclipse.swt.widgets.Composite;
import org.eclipse.swt.widgets.Display;

import cast.ui.architecture.ComponentMonitor;
import cast.ui.architecture.SubarchitectureMonitor;

/**
 * @author nah
 */
public class SubarchitectureView extends ArchitectureViewGroup {

    private SubarchitectureMonitor m_subarchitectureMonitor;
    private ArrayList<ComponentView> m_componentViews;

    /**
     * @param _parent
     */
    public SubarchitectureView(SubarchitectureMonitor _subarchMon,
            Display _display, Composite _parent) {
        super(_subarchMon.getID(), _parent, SWT.NO_REDRAW_RESIZE,
            _display);
        m_subarchitectureMonitor = _subarchMon;

        GridLayout layout = new GridLayout(2, true);
        setLayout(layout);

        // GridData gd = new GridData(GridData.FILL_BOTH);
        GridData gd = new GridData(GridData.CENTER);
        // GridData gd = new GridData(SWT.FILL, SWT.FILL,
        // true, true);

        m_componentViews = new ArrayList<ComponentView>();

        ComponentView tmcv = new ComponentView(m_subarchitectureMonitor
            .getTaskManager(), _display, this);
        tmcv.addReaderView();
        tmcv.setLayoutData(gd);
        m_componentViews.add(tmcv);

        ComponentView wmcv = new ComponentView(m_subarchitectureMonitor
            .getWorkingMemory(), _display, this);
        wmcv.addWriterView();
        wmcv.addReaderView();
        wmcv.setLayoutData(gd);
        m_componentViews.add(wmcv);

        for (ComponentMonitor monitor : m_subarchitectureMonitor
            .getDataDrivenComponents()) {
            ComponentView cv = new ComponentView(monitor, _display,
                this);
            cv.addWriterView();
            cv.setLayoutData(gd);
            m_componentViews.add(cv);
        }

        for (ComponentMonitor monitor : m_subarchitectureMonitor
            .getGoalDrivenComponents()) {
            ComponentView cv = new ComponentView(monitor, _display,
                this);
            cv.addWriterView();
            cv.addReaderView();
            cv.addManagedView();
            cv.setLayoutData(gd);
            m_componentViews.add(cv);
        }

        // addPaintListener(new PaintListener() {
        // public void paintControl(PaintEvent _e) {
        // drawView(_e);
        // }
        // });

        pack(true);
    }

//    /*
//     * (non-Javadoc)
//     * 
//     * @see caat.ui.architecture.views.ArchitectureViewGroup#drawView(org.eclipse.swt.events.PaintEvent)
//     */
//    @Override
//    protected void drawView() {
//        // System.out.println("SubarchitectureView.drawView()");
////        if (!CAATProcessServer.m_os.equals(CAATProcessServer.MAC_OS_X)) {
////            System.out
////                .println("SubarchitectureView.drawView() calling subdraws");
////            for (ComponentView cv : m_componentViews) {
////                cv.drawView();
////            }
////        }
//
//    }

    @Override
    public void updateView() {
        for (ComponentView cv : m_componentViews) {
            cv.updateView();
        }
    }

}
