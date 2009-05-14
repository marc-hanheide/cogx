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
package cast.core.components;

import java.util.Properties;

import cast.cdl.SUBARCH_ID_KEY;
import cast.core.CASTUtils;

/**
 * A class to provide additional functionality for those components that
 * are procesing components in a CAAT architecture.
 * 
 * @author nah
 */
public abstract class CASTProcessingComponent extends CASTUIComponent {

    static {
        CASTUtils.initialiseCAST();
    }


    // the id of the subarchitecture that contains this component
    protected String m_subarchitectureID;

    /**
     * Construct a new processing component with the given unique ID.
     * 
     * @param _id
     *            The id used to identify this component.
     */
    public CASTProcessingComponent(String _id) {
        super(_id);
        m_subarchitectureID = null;
        // println("Constuctor");
    }

    /**
     * Overrides the configure method from FrameworkProcess to use
     * _config to set the subarchitecture ID.
     * 
     * @see framework.core.processes.FrameworkProcess#configure(String)
     * @param _config
     *            The ID of the subarchitecture which contains this
     *            component.
     */
    @Override
    public void configure(Properties _config) {
        super.configure(_config);

        // println("CAATComponent.configure(): " + _config);
        m_subarchitectureID = _config.getProperty(SUBARCH_ID_KEY.value);

        if (m_subarchitectureID == null) {
            println("Error, configure property not found: "
                + SUBARCH_ID_KEY.value);
        }

    }

    /**
     * @return
     */
    public final String getSubarchitectureID() {
        return m_subarchitectureID;
    }
    
}
