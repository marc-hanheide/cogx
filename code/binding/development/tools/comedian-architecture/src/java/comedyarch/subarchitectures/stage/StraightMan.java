/*
 * Comedian example code to demonstrate CAST functionality.
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
package comedyarch.subarchitectures.stage;

import cast.architecture.subarchitecture.SubarchitectureProcessException;
import cast.architecture.subarchitecture.UnmanagedProcess;

import comedyarch.autogen.Joke;

/**
 * @author nah
 */
public class StraightMan extends UnmanagedProcess {

    /**
     * @param _id
     */
    public StraightMan(String _id) {
        super(_id);

        // set the ontology for this method
//        setOntology(ComedyOntologyFactory.getOntology());

    }

    private String generateSetup() {
        return "What's brown and sticky?";
    }

    /*
     * (non-Javadoc)
     * 
     * @see cast.core.components.CASTComponent#runComponent()
     */
    @Override
    public void runComponent() {
        try {
            while (m_status == ProcessStatus.RUN) {
                // do nothing for a while
                Thread.sleep(5000);

                // must check we're still running after sleep!
                if (m_status == ProcessStatus.RUN) {

                    // lock from external access
                    lockProcess();

                    println("ahem...");

                    // make up a joke
                    Joke jk = new Joke(generateSetup(), "");

                    // and then make it available in the s-a working
                    // memory
                    addToWorkingMemory(newDataID(), jk);

                    // let other stuff happen if necessary
                    unlockProcess();
                }
            }
        }
        catch (InterruptedException e) {
            e.printStackTrace();
        }
        catch (SubarchitectureProcessException e) {
            e.printStackTrace();
        }

    }

}
