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
import org.eclipse.swt.layout.GridLayout;
import org.eclipse.swt.widgets.Display;

import cast.ui.architecture.*;

/**
 * @author nah
 */
public class ComponentView extends ArchitectureViewGroup implements
        ComponentMonitorListener {

    private ComponentMonitor m_componentMonitor;
    private ArrayList<ComponentViewAnnotator> m_annotators;

    /**
     * @param _parent
     * @param _style
     */
    public ComponentView(ComponentMonitor _subarchMon,
            Display _display, SubarchitectureView _parent) {

        super(_subarchMon.getID(), _parent, SWT.NO_REDRAW_RESIZE,
            _display);

        m_componentMonitor = _subarchMon;
        m_componentMonitor.addComponentMonitorListener(this);

        m_annotators = new ArrayList<ComponentViewAnnotator>(1);

        GridLayout layout = new GridLayout(2, true);
        setLayout(layout);

        // m_group.setRedraw(false);
    }

    /*
     * (non-Javadoc)
     * 
     * @see caat.ui.architecture.ComponentMonitorListener#componentMonitorChanged(caat.ui.architecture.ComponentMonitorChangeEvent)
     */
    public void componentMonitorChanged(ComponentMonitorChangeEvent _evt) {
    // System.out.println(UIUtils.toString(m_componentMonitor.getStatus()));
    // signalRedraw();
    }

    public void addReaderView() {
        m_annotators.add(new ReaderViewAnnotator(getGroup(),
            m_componentMonitor));
        pack(true);
        // System.out.println(getSize().y);
    }

    public void addWriterView() {
        m_annotators.add(new WriterViewAnnotator(getGroup(),
            m_componentMonitor));
        pack(true);
        // System.out.println(getSize().y);
    }

    public void addManagedView() {
        m_annotators.add(new ManagedViewAnnotator(getGroup(),
            m_componentMonitor));
        pack(true);
        // System.out.println(getSize().y);
    }

    // /* (non-Javadoc)
    // * @see org.eclipse.swt.widgets.Group#computeSize(int, int,
    // boolean)
    // */
    // @Override
    // public Point computeSize(int _arg0, int _arg1, boolean _arg2) {
    // return new Point(100,200);
    // }
    //    
    // /* (non-Javadoc)
    // * @see org.eclipse.swt.widgets.Control#computeSize(int, int)
    // */
    // @Override
    // public Point computeSize(int _arg0, int _arg1) {
    // return new Point(100,200);
    // }

//    /*
//     * (non-Javadoc)
//     * 
//     * @see caat.ui.architecture.views.ArchitectureViewGroup#redraw()
//     */
//    public void redraw() {
//        System.out.println("ComponentView.redraw()"
//            + m_componentMonitor.getID());
//        drawView();
//    }

//    /*
//     * (non-Javadoc)
//     * 
//     * @see caat.ui.architecture.views.ArchitectureViewGroup#drawView(org.eclipse.swt.events.PaintEvent)
//     */
//    @Override
//    protected void drawView() {
////        System.out.println("ComponentView.drawView() "
////            + m_componentMonitor.getID());
////        // m_group.setRedraw(true);
////        for (ComponentViewAnnotator annotator : m_annotators) {
////            annotator.update();
////        }
////        // m_group.setRedraw(false);
//    }

    @Override
    public void updateView() {
        for (ComponentViewAnnotator annotator : m_annotators) {
            annotator.update();
        }
    }
}
