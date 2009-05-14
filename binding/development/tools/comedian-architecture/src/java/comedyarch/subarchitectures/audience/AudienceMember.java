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
package comedyarch.subarchitectures.audience;

import java.util.Hashtable;
import java.util.Properties;

import cast.architecture.abstr.ChangeFilterFactory;
import cast.architecture.abstr.WorkingMemoryChangeReceiver;
import cast.architecture.subarchitecture.PrivilegedManagedProcess;
import cast.architecture.subarchitecture.SubarchitectureProcessException;
import cast.cdl.OperationMode;
import cast.cdl.TaskOutcome;
import cast.cdl.WorkingMemoryChange;
import cast.cdl.WorkingMemoryOperation;
import cast.core.data.CASTData;

import comedyarch.autogen.Joke;
import comedyarch.global.ComedyGoals;

/**
 * @author nah
 */
public class AudienceMember extends PrivilegedManagedProcess {

    // Hashtable used to record the tasks we want to carry out
    private Hashtable<String, CASTData> m_proposedProcessing;
    private String m_defaultReaction;

    /**
     * @param _id
     */
    public AudienceMember(String _id) {
        super(_id);
        m_proposedProcessing = new Hashtable<String, CASTData>();
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
            addChangeFilter(ChangeFilterFactory.createGlobalTypeFilter(Joke.class, WorkingMemoryOperation.ADD),
                new WorkingMemoryChangeReceiver() {

                    public void workingMemoryChanged(WorkingMemoryChange _wmc) {
                        newJokeAdded(_wmc);
                    }
                });
        }
        catch (SubarchitectureProcessException e) {
            // TODO Auto-generated catch block
            e.printStackTrace();
            System.exit(1);
        }
    }

    /*
     * (non-Javadoc)
     * 
     * @see cast.architecture.subarchitecture.ManagedProcess#taskAdopted(java.lang.String)
     */
    @Override
    protected void taskAdopted(String _goalID) {
        // wooo! we're allowed to act

        // get the data we stored for this goal...
        CASTData data = m_proposedProcessing.remove(_goalID);

        // if we have stored this goal earlier
        if (data != null) {
            // well... let's see
            Joke joke = (Joke) data.getData();
            // this is what I think
            String reaction = generateAudienceReaction(joke);

            try {

                String id = newDataID();

                // let's tell the world!
                addToWorkingMemory(id, m_subarchitectureID, 
                    reaction, OperationMode.BLOCKING); //take a sync approach to guarantee it's there

//                //must run with -ea flag to enable asserts in java 
//                assert(existsOnWorkingMemory(id));                
                

            }
            catch (SubarchitectureProcessException e) {
                e.printStackTrace();
            }
        }
        else {
            println("oh, this is my goal, but I have no data: "
                + _goalID);
        }

        // and now we're finished, tell the goal manager that the task
        // is over successfully (assuming it is!)
        try {
            taskComplete(_goalID,
                TaskOutcome.PROCESSING_COMPLETE_SUCCESS);
        }
        catch (SubarchitectureProcessException e) {
            e.printStackTrace();
        }
    }

    /*
     * (non-Javadoc)
     * 
     * @see cast.core.components.CASTProcessingComponent#configure(java.util.Properties)
     */
    @Override
    public void configure(Properties _config) {

//        _config.list(System.out);

        super.configure(_config);

        if (_config.containsKey("--reaction")) {
            m_defaultReaction = _config.getProperty("--reaction");
        }
        else {
            m_defaultReaction = "BOO!";
        }
    }

    private String generateAudienceReaction(Joke _joke) {
        return m_defaultReaction;
    }

    /*
     * (non-Javadoc)
     * 
     * @see cast.architecture.subarchitecture.ManagedProcess#taskRejected(java.lang.String)
     */
    @Override
    protected void taskRejected(String _goalID) {
        println(":(");
        m_proposedProcessing.remove(_goalID);
    }

    /**
     * @param _wmc
     * @param i
     */
    @SuppressWarnings("unchecked")
    private void newJokeAdded(WorkingMemoryChange _wmc) {
        // get the joke from working memory to evaluate
        try {

            // get the id of the working memory entry
            String id = _wmc.m_address.m_id;

            if (_wmc.m_address.m_subarchitecture
                .equals(m_subarchitectureID)) {
                // get the data from working memory and store it
                // with its id
                CASTData jokeData = getWorkingMemoryEntry(id);

                println("(takes deep breath)");

                // get a new id for the task
                String taskID = newTaskID();

                // store the data we want to process for
                // later... we could just store the id in
                // working memory if we wanted to, I guess it
                // depends on storage/transport tradeoffs
                m_proposedProcessing.put(taskID, jokeData);

                // then ask for permission
                proposeInformationProcessingTask(taskID,
                    ComedyGoals.RESPOND_TO_JOKE_TASK);
            }
            else {
//                CASTData<Joke> jokeData =
//                        (CASTData<Joke>) getWorkingMemoryEntry(
//                            id, _wmc.m_address.m_subarchitecture);
//                log("sneaky peak");
//                log(jokeData.getData().m_setup);
//                log(jokeData.getData().m_punchline);
            }

        }
        catch (SubarchitectureProcessException e) {
            e.printStackTrace();
        }
    }

    /*
     * (non-Javadoc)
     * 
     * @see cast.core.components.CASTComponent#runComponent()
     */
    @Override
    public void runComponent() {
    // do nothing here
    }

}
