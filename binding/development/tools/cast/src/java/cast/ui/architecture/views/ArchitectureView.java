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
import java.util.Collection;

import org.eclipse.swt.SWT;
import org.eclipse.swt.layout.GridData;
import org.eclipse.swt.layout.GridLayout;
import org.eclipse.swt.widgets.*;

import cast.ui.architecture.ArchitectureMonitor;
import cast.ui.architecture.SubarchitectureMonitor;

/**
 * @author nah
 */
public class ArchitectureView extends ArchitectureViewGroup {

    private ArchitectureMonitor m_architectureMonitor;
    private int m_subarchsPerRow = 3;
    private ArrayList<SubarchitectureView> m_subarchitectureViews;

    /**
     * @param _parent
     * @param _style
     */
    public ArchitectureView(ArchitectureMonitor _archMon,
            Display _display, Composite _parent) {

        // super(_parent, SWT.NO_REDRAW_RESIZE);
        super("architecture", _parent, SWT.NO_REDRAW_RESIZE, _display);

        m_architectureMonitor = _archMon;

        // addPaintListener(new PaintListener() {
        //
        // public void paintControl(PaintEvent _e) {
        // drawView(_e);
        // }
        // });

        Collection<SubarchitectureMonitor> subarchictectures = m_architectureMonitor
            .getSubarchitectures();

        int subarchCount = subarchictectures.size();

        GridLayout layout;
        if (subarchCount >= m_subarchsPerRow) {
            layout = new GridLayout(m_subarchsPerRow, true);
        }
        else {
            layout = new GridLayout(subarchCount, false);
        }

        setLayout(layout);

        // GridData gd = new GridData(GridData.FILL_BOTH);
        GridData gd = new GridData(GridData.CENTER);
        // GridData gd = new GridData(SWT.FILL, SWT.FILL,
        // true, true);

        Collection<SubarchitectureMonitor> subarchitectureMonitors = m_architectureMonitor
            .getSubarchitectures();
        m_subarchitectureViews = new ArrayList<SubarchitectureView>(
            subarchitectureMonitors.size());
        for (SubarchitectureMonitor monitor : subarchitectureMonitors) {
            // SubarchitectureView sv = new
            // SubarchitectureView(monitor,_display,this,
            // SWT.NO_REDRAW_RESIZE);
            SubarchitectureView sv = new SubarchitectureView(monitor,
                _display, getGroup());
            sv.setLayoutData(gd);
            m_subarchitectureViews.add(sv);
        }

        pack(true);

    }

//    /*
//     * (non-Javadoc)
//     * 
//     * @see caat.ui.architecture.views.ArchitectureViewGroup#drawView(org.eclipse.swt.events.PaintEvent)
//     */
//    @Override
//    protected void drawView() {
//
//    // System.out.println();
//    // System.out.println("ArchitectureView.drawView()");
//
//    // m_group.setRedraw(true);
//    // if (!CAATProcessServer.m_os.equals(CAATProcessServer.MAC_OS_X)) {
//    // System.out.println("ArchitectureView.drawView() calling
//    // subdraws");
//    // for (SubarchitectureView sv : m_subarchitectureViews) {
//    // sv.drawView();
//    // }
//    // }
//    // m_group.setRedraw(false);
//    }

    @Override
    public void updateView() {

        for (SubarchitectureView sv : m_subarchitectureViews) {
            sv.updateView();
        }
    }

    /**
     * @return
     */
    public Control getView() {
        return getGroup();
    }

}
