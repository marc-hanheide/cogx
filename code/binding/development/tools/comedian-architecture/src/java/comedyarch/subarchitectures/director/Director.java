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
package comedyarch.subarchitectures.director;

import java.util.Properties;

import cast.architecture.abstr.ChangeFilterFactory;
import cast.architecture.abstr.WorkingMemoryChangeReceiver;
import cast.architecture.subarchitecture.PrivilegedManagedProcess;
import cast.architecture.subarchitecture.SubarchitectureProcessException;
import cast.cdl.WorkingMemoryAddress;
import cast.cdl.WorkingMemoryChange;
import cast.cdl.WorkingMemoryOperation;
import cast.cdl.testing.CAST_TEST_FAIL;
import cast.cdl.testing.CAST_TEST_PASS;
import cast.core.data.CASTData;

import comedyarch.autogen.DirectorAction;
import comedyarch.autogen.DirectorActionType;
import comedyarch.autogen.Joke;

/**
 * @author nah
 */
public class Director extends PrivilegedManagedProcess {

    private boolean m_testing;

	/**
     * @param _id
     */
    public Director(String _id) {
        super(_id);
//        setOntology(ComedyOntologyFactory.getOntology());
    }

    /*
     * (non-Javadoc)
     * 
     * @see cast.architecture.abstr.WorkingMemoryReaderProcess#start()
     */
    @Override
    public void start() {
        super.start();

        try {
            addChangeFilter(ChangeFilterFactory.createLocalTypeFilter(DirectorAction.class, WorkingMemoryOperation.ADD),
                new WorkingMemoryChangeReceiver() {

                    public void workingMemoryChanged(WorkingMemoryChange _wmc) {
//                        log(CASTUtils.toString(_wmc));
                        takeAction(_wmc);
                    }
                });
        }
        catch (SubarchitectureProcessException e) {
            e.printStackTrace();
        }

    }

    /**
     * @param _address
     */
    private void takeAction(WorkingMemoryChange _actionChange) {
        // get the action from wm
        try {
            CASTData<?> wme = getWorkingMemoryEntry(_actionChange.m_address);
            DirectorAction action = (DirectorAction) wme.getData();
            switch (action.m_action.value()) {
                case DirectorActionType._AskTheAudience:
                    askTheAudience(action.m_address);
                    break;
                case DirectorActionType._CheckTheReaction:
                    checkTheReaction(action.m_address);
                    break;
                default:
                    log("Unknown action type");
                    break;
            }

            // // now delete the action itself from our sa wm
            // debug("deleting action at: " + CASTUtils.toString(_wma));
            deleteFromWorkingMemory(_actionChange.m_address.m_id);
        }
        catch (SubarchitectureProcessException e) {
            e.printStackTrace();
        }
    }

    /**
     * @param _reactionAddress
     * @throws SubarchitectureProcessException
     */
    private void checkTheReaction(WorkingMemoryAddress _reactionAddress)
            throws SubarchitectureProcessException {
   
        // get the finished masterpiece
        CASTData<?> reactionData = getWorkingMemoryEntry(_reactionAddress);
        String reaction = (String) reactionData.getData();

        println("and the audience says...");
        println(reaction);

        // and erase it from the memory of the poor jokers
        deleteFromWorkingMemory(_reactionAddress.m_id,
            _reactionAddress.m_subarchitecture);
        
        
  if(m_testing) {
        if(reaction.equals("YAY!")) {
            System.exit(CAST_TEST_PASS.value);
          }
          else {
        	  System.exit(CAST_TEST_FAIL.value);
          }
        }
}

    /**
     * @param _jokeAddress
     * @throws SubarchitectureProcessException
     */
    private void askTheAudience(WorkingMemoryAddress _jokeAddress)
            throws SubarchitectureProcessException {

//        assert(existsOnWorkingMemory(_jokeAddress));
        
        println("someone told a joke! ");

        // get the finished masterpiece
        CASTData jokeData = getWorkingMemoryEntry(_jokeAddress);
        Joke joke = (Joke) jokeData.getData();

        println("let's see what the audience thinks of...");
        println("Q: " + joke.m_setup);
        println("A: " + joke.m_punchline);

        // write it to the audience subarchitecture
        addToWorkingMemory(newDataID(), "audience.subarch",
            joke);

        // and erase it from the memory of the poor jokers
        deleteFromWorkingMemory(_jokeAddress.m_id,
            _jokeAddress.m_subarchitecture);
    }

    @Override
    public void configure(Properties _config) {
    	super.configure(_config);
     	 if(_config.containsKey("--test")) {
     		 m_testing = true;
     	 }
     	 else {
     		 m_testing = false;
     	 }
    }
    
    /*
     * (non-Javadoc)
     * 
     * @see cast.architecture.subarchitecture.PrivilegedManagedProcess#taskAdopted(java.lang.String)
     */
    @Override
    protected void taskAdopted(String _taskID) {}

    /*
     * (non-Javadoc)
     * 
     * @see cast.architecture.subarchitecture.PrivilegedManagedProcess#taskRejected(java.lang.String)
     */
    @Override
    protected void taskRejected(String _taskID) {}

}
