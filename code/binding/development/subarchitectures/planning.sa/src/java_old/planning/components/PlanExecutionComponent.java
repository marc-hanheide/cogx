/**
 * 
 */
package planning.components;


/**
 * Problems: Grounding and keeping track of waypoints, particularly the
 * deduced target points. How to translate to
 * 
 * @author nah
 */
public class PlanExecutionComponent {
//        extends
//            PlanningSubarchitectureProcess {
//
//    protected Hashtable<String, String> m_queryMap;
//    protected float m_spatialRelThreshold;
//
//    protected Hashtable<String, PlanExecutionData> m_taskMap;
//
//    protected String m_visualSubarchitectureID;
//
//    // YUUUUUUUUUCCCCCCCKKKKKKKKK
//    protected Hashtable<String, ArrayList<String>> m_waypointToQueryMap;
//
//    // protected boolean m_bIgnoreValidation = false;
//    protected boolean m_bIgnoreValidation = true;
//
//    /**
//     * @param _id
//     */
//    public PlanExecutionComponent(String _id) {
//        super(_id);
//
//        m_taskMap = new Hashtable<String, PlanExecutionData>();
//        m_queryMap = new Hashtable<String, String>();
//        m_waypointToQueryMap = new Hashtable<String, ArrayList<String>>();
//
//        setReceiveXarchChangeNotifications(true);
//
//        addChangeFilter(PlanningOntology.PLAN_TYPE,
//            WorkingMemoryOperation.ADD);
//
//        addChangeFilter(
//            SpatialOntology.SPATIAL_RELATIONSHIP_RESPONSE_TYPE,
//            WorkingMemoryOperation.ADD);
//
//        addChangeFilter(
//            ManipulationOntology.PICK_AND_PLACE_COMMAND_TYPE,
//            WorkingMemoryOperation.OVERWRITE);
//
//        m_spatialRelThreshold = 0.7f;
//    }
//
//    /**
//     * @param exeData
//     * @throws SubarchitectureProcessException
//     */
//    private boolean advancePlanInPlanMemory(PlanExecutionData exeData) {
//        log("advancing one step");
//
//        String step = m_plannerServer.get_next_step(exeData
//            .getPlanMemoryID());
//
//        if (step.equals("")) {
//            log("plan execution complete"); // won't actually happen in
//            // this setup
//            return true;
//        }
//        else {
//            m_plannerServer.advance(exeData.getPlanMemoryID(), step);
//            return false;
//        }
//    }
//
//    /**
//     * @param _bindingAddress
//     * @return
//     * @throws SubarchitectureProcessException
//     * @throws SpatialSubarchitectureException
//     */
//    private InstanceBinding getInstanceBinding(
//            WorkingMemoryAddress _bindingAddress)
//            throws SubarchitectureProcessException {
//        CASTTypedData bindingData = getWorkingMemoryEntry(
//            _bindingAddress.m_subarchitecture, _bindingAddress.m_id);
//
//        if (bindingData == null) {
//            throw new PlanningException(
//                "Binding no longer exists: "
//                    + _bindingAddress.m_subarchitecture + ":"
//                    + _bindingAddress.m_id);
//        }
//
//        InstanceBinding instanceBinding = (InstanceBinding) bindingData
//            .getData();
//        return instanceBinding;
//    }
//
//    /**
//     * @param _address
//     * @return
//     * @throws SubarchitectureProcessException
//     */
//    private SpatialRelationshipTargetResponse getTargetResponse(
//            WorkingMemoryAddress _address)
//            throws SubarchitectureProcessException {
//        CASTTypedDataWithID data = getWorkingMemoryEntry(
//            _address.m_subarchitecture, _address.m_id);
//        return (SpatialRelationshipTargetResponse) data.getData();
//    }
//
//    /**
//     * Get metric info for waypoints
//     * 
//     * @param _exeData
//     * @throws SubarchitectureProcessException
//     */
//    private String groundWaypoints(PlanExecutionData _exeData)
//            throws SubarchitectureProcessException {
//
//        String newMapID = null;
//
//        Iterator<String> wpIter = _exeData.waypointIDIterator();
//        String wpID;
//
//        // first pass, ground the waypoints we know about
//        while (wpIter.hasNext()) {
//            wpID = wpIter.next();
//            // find out if an object is at the position
//            String spatialObjectID = _exeData.getObjectAtWaypoint(wpID);
//            if (spatialObjectID != null) {
//                SpatialObject so = getSpatialObject(spatialObjectID);
//                _exeData
//                    .setWaypointPosition(wpID, so.m_pose.m_position);
//                log(wpID + " grounded via object reference: "
//                    + so.m_bbox.m_centroid.m_x + " "
//                    + so.m_bbox.m_centroid.m_x);
//            }
//        }
//
//        // second pass, ground the ones that are in relation to the ones
//        // we know
//
//        // ANOTHR HACK... promise I'll clean this all up later GJ
//        Hashtable<String, ArrayList<String>> m_relsMap = new Hashtable<String, ArrayList<String>>();
//        wpIter = _exeData.waypointIDIterator();
//        while (wpIter.hasNext()) {
//            wpID = wpIter.next();
//            // log(wpID);
//            if (_exeData.getWaypointPosition(wpID) == null) {
//                // Uh oh, need to query something
//
//                // go through the fact list to find the related
//                // waypoints
//                String[] facts = _exeData.getCurrentPlanFacts();
//                for (int i = 0; i < facts.length; i++) {
//                    String fact = facts[i];
//                    // log(fact);
//                    String[] factParts = fact.split(" ");
//                    // find one with our wp in
//                    if (factParts.length == 3
//                        && factParts[1].equals(wpID)) {
//                        // check that it's related to a properly
//                        // grounded wp
//                        if (_exeData.getWaypointPosition(factParts[2]) != null) {
//                            // at this point we can add it to the query
//                            String relWpID = factParts[0]
//                                + factParts[2];
//
//                            // log(relWpID);
//
//                            if (m_relsMap.containsKey(relWpID)) {
//                                m_relsMap.get(relWpID)
//                                    .add(factParts[1]); // additional
//                                // landmarks
//                                // log("addding more");
//                            }
//                            else {
//                                // log("addding first");
//                                ArrayList<String> dirtyDirty = new ArrayList<String>();
//                                dirtyDirty.add(factParts[0]); // the
//                                // spatial
//                                // rel
//                                dirtyDirty.add(_exeData
//                                    .getObjectAtWaypoint(factParts[2])); // the
//                                // landmark
//                                dirtyDirty.add(factParts[1]); // the
//                                // first
//                                // target
//                                m_relsMap.put(relWpID, dirtyDirty);
//                            }
//                        }
//                    }
//                }
//            }
//        }
//
//        // at this point we should be able to see what the required rels
//        // look like
//        Iterator<String> iter = m_relsMap.keySet().iterator();
//
//        while (iter.hasNext()) {
//
//            // if we're doing this then we're going to have to store
//            // everything a bit longer
//            if (newMapID == null) {
//                newMapID = newDataID();
//            }
//
//            ArrayList<String> queryList = m_relsMap.get(iter.next());
//
//            if (m_bLogOutput) {
//                String out = "querying: ";
//                for (String string : queryList) {
//                    out += string + " ";
//                }
//                log(out);
//            }
//
//            // take off 2 for prep and landmark
//            int targetCount = queryList.size() - 2;
//            String landmarkID = queryList.get(1);
//
//            // HACK count how many times you write this chump
//            String relString = queryList.get(0);
//            SpatialRelationshipType relType;
//
//            relType = string2SpatialRelationshipType(relString);
//
//            SpatialRelationshipTargetQuery query = new SpatialRelationshipTargetQuery(
//                relType, landmarkID, targetCount,
//                new WorkingMemoryAddress("", "")); // current scene is
//            // empty address
//            String queryID = newDataID();
//            m_queryMap.put(queryID, newMapID);
//            m_waypointToQueryMap.put(queryID, queryList);
//
//            // add query to working memory
//            addToWorkingMemory(queryID,
//                SpatialOntology.SPATIAL_RELATIONSHIP_QUERY_TYPE, query);
//
//        }
//
//        return newMapID;
//
//    }
//
//    /**
//     * @param _lfGoal
//     * @throws SubarchitectureProcessException
//     */
//    private void queuePlan(PlanExecutionData _plan)
//            throws SubarchitectureProcessException {
//        // log("queuing plan: " +
//        // Arrays.toString(_plan.getPlanSteps()));
//        String taskID = newTaskID();
//        m_taskMap.put(taskID, _plan);
//        proposeInformationProcessingTask(taskID,
//            PlanningGoals.EXECUTE_PLAN_STEP);
//    }
//
//    /**
//     * @param _exeData
//     * @throws SubarchitectureProcessException
//     */
//    private void triggerExecution(PlanExecutionData _exeData)
//            throws SubarchitectureProcessException {
//        // this triggers the actual execution of the next step in the
//        // plan
//
//        String[] planSteps = _exeData.getPlanSteps();
//
//        if (planSteps.length >= 2) {
//            log("triggering execution of paired step: " + planSteps[0]);
//            log("triggering execution of paired step: " + planSteps[1]);
//
//            String[] pickStep = planSteps[0].split(" ");
//            String[] placeStep = planSteps[1].split(" ");
//
//            if (pickStep.length != 4 && !pickStep[0].equals("pickup")) {
//                throw new PlanningException(
//                    "incorrect pickup action: " + planSteps[0]);
//            }
//            else if (placeStep.length != 4
//                && !placeStep[0].equals("put")) {
//                throw new PlanningException(
//                    "incorrect put action: " + planSteps[1]);
//            }
//
//            // ok let's go then!!!
//
//            // need visual object from spatial object... blimey!!
//            SpatialObject so = getSpatialObject(pickStep[2]);
//            InstanceBinding ib = getInstanceBinding(so.m_address);
//            WorkingMemoryAddress wma = getSceneObjectAddressViaBinding(ib);
//
//            Pose3D target = _exeData.getWaypointPosition(placeStep[3]);
//
//            PickAndPlaceCmd papc = new PickAndPlaceCmd(wma.m_id,
//                target, 0);
//
//            log("triggering execution of PickAndPlaceCmd: "
//                + papc.m_objId + " ("
//                + papc.m_targetPose.m_position.m_x + ","
//                + papc.m_targetPose.m_position.m_y + ","
//                + papc.m_targetPose.m_position.m_z + ")");
//
//            String cmdID = newDataID();
//            // store the task
//            m_taskMap.put(cmdID, _exeData);
//            addToWorkingMemory(cmdID,
//                ManipulationOntology.PICK_AND_PLACE_COMMAND_TYPE, papc);
//
//        }
//        else {
//            throw new PlanningException(
//                "plan in inconsistent state, actions should be paired: "
//                    + Arrays.toString(planSteps));
//        }
//
//    }
//
//    /**
//     * @param _exeData
//     * @throws SubarchitectureProcessException
//     */
//    private MonitorOutput validatePlan(PlanExecutionData _exeData)
//            throws SubarchitectureProcessException {
//
//        // cheat for now!
//
//        // // get current scene
//        ArrayList<ObjectDeclaration> maplObjects = new ArrayList<ObjectDeclaration>();
//        ArrayList<String> maplFacts = new ArrayList<String>();
//        // just pretend stuff for not
//        SpatialScene currentScene = getCurrentScene();
//        if (currentScene == null) {
//            println("no current scene for planning to work with");
//            return null;
//        }
//
//        log("planning working with scene: "
//            + SpatialUtils.toString(currentScene));
//
//        ArrayList<CentralRepresentationPair> objects = getStateRepresentation(currentScene);
//
//        spatialSceneToMAPL(currentScene, objects, maplObjects,
//            maplFacts);
//
//        ObjectDeclaration[] extraObjs = _exeData.getInitialState().m_extraObjectList;
//        for (int i = 0; i < extraObjs.length; i++) {
//            maplObjects.add(extraObjs[i]);
//        }
//
//        String[] extraFacts = _exeData.getInitialState().m_extraFactList;
//        for (int i = 0; i < extraFacts.length; i++) {
//            maplFacts.add(extraFacts[i]);
//        }
//
//        // _exeData.setCurrentPlanFacts(m_plannerServer
//        // .current_state(_exeData.getPlanMemoryID()));
//
//        // set world state as current state
//        _exeData
//            .setCurrentPlanObjects((ObjectDeclaration[]) maplObjects
//                .toArray(new ObjectDeclaration[maplObjects.size()]));
//        _exeData.setCurrentPlanFacts((String[]) maplFacts
//            .toArray(new String[maplFacts.size()]));
//
//        // take the next step in the plan
//
//        MonitorOutput result = new MonitorOutput(true, true, "");
//
//        _exeData.setPlanSteps(m_plannerServer.current_plan(_exeData
//            .getPlanMemoryID()));
//
//        if (!m_bIgnoreValidation) {
//
//            log("Monitoring Objects:");
//            for (Iterator iter = maplObjects.iterator(); iter.hasNext();) {
//                ObjectDeclaration od = (ObjectDeclaration) iter.next();
//                log("  " + od.name + " : " + od.type);
//            }
//
//            log("Monitoring Facts:");
//            for (Iterator iter = maplFacts.iterator(); iter.hasNext();) {
//                log("  " + iter.next());
//            }
//
//            log("Remaining plan:\n  "
//                + Arrays.toString(_exeData.getPlanSteps()));
//
//            log("Current state:\n  "
//                + Arrays.toString(_exeData.getCurrentPlanFacts()));
//
//            PlanningProblem initialState = _exeData.getInitialState();
//
//            try {
//                result = m_plannerServer.validate_plan(_exeData
//                    .getPlanSteps(),
//                // m_intialProblem.m_objectList, facts,
//                    initialState.m_objectList, _exeData
//                        .getCurrentPlanFacts(),
//                    initialState.m_maplGoal, initialState.m_domainFile);
//            }
//            catch (Failure e) {
//                System.out.println("Failure");
//            }
//
//            log("Is the plan executable in the current state? "
//                + result.executable);
//            log("Does the plan achieve the goal? "
//                + result.goal_achieved);
//        }
//        return result;
//
//        // if (plan.length == 0) {
//        // if (result.goal_achieved) {
//        // goalReached = true;
//        // }
//        // break;
//        // }
//        //
//        // String step = m_plannerServer.get_next_step(m_pmID);
//        // System.out.println("Executing next step: " + step);
//
//    }
//
//    /**
//     * @param _bindingAddress
//     * @return
//     * @throws SubarchitectureProcessException
//     */
//    protected WorkingMemoryAddress getSceneObjectAddressViaBinding(
//            InstanceBinding _instanceBinding)
//            throws SubarchitectureProcessException {
//
//        // HACK If the binding is not visible, then do nothing
//        String visiblity = BindingUtils.getBindingFeature(
//            _instanceBinding, BindingPrimitiveFeatureEnum.VISIBLE)[0];
//
//        if (visiblity == null
//            || visiblity.equals(BindingUtils.FALSE_FEATURE_VALUE)) {
//            log("instance binding is not currently visible: "
//                + BindingUtils.toString(_instanceBinding));
//            return null;
//        }
//
//        return getVisualSubarchitectureBinding(_instanceBinding);
//
//    }
//
//    /**
//     * @param _binding
//     * @param _subarchID
//     * @return
//     * @throws SubarchitectureProcessException
//     */
//    protected WorkingMemoryAddress getSubarchitectureBinding(
//            InstanceBinding _binding, String _subarchID)
//            throws SubarchitectureProcessException {
//        WorkingMemoryAddress[] ecs = _binding.m_bindings;
//        for (int i = 0; i < ecs.length; i++) {
//            // log("subarch check: "+ ecs[i].m_subarchitecture + " == "
//            // + _subarchID);
//            WorkingMemoryAddress candidateAddress = ecs[i];
//
//            // get the candidate
//            InstanceBindingCandidate candidate = (InstanceBindingCandidate) getWorkingMemoryEntry(
//                candidateAddress.m_subarchitecture,
//                candidateAddress.m_id).getData();
//
//            if (candidate.m_candidateAddress.m_subarchitecture
//                .equals(_subarchID)) {
//                return candidate.m_candidateAddress;
//            }
//        }
//        return null;
//    }
//
//    /**
//     * @param _binding
//     * @return
//     * @throws SubarchitectureProcessException
//     */
//    protected WorkingMemoryAddress getVisualSubarchitectureBinding(
//            InstanceBinding _binding)
//            throws SubarchitectureProcessException {
//        return getSubarchitectureBinding(_binding,
//            m_visualSubarchitectureID);
//    }
//
//    /*
//     * (non-Javadoc)
//     * 
//     * @see cast.architecture.subarchitecture.ManagedProcess#taskAdopted(java.lang.String)
//     */
//    @Override
//    protected void taskAdopted(String _goalID) {
////        PlanExecutionData exeData = m_taskMap.remove(_goalID);
////
////        try {
////            if (exeData == null) {
////                println("received unknown goal rejection: " + _goalID);
////                return;
////            }
////
////            // check the next step in the plan
////            //
////            //
////            MonitorOutput success = validatePlan(exeData);
////
////            if (m_bIgnoreValidation
////                || (success.executable && success.goal_achieved)) {
////
////                if (exeData.getPlanSteps().length == 0) {
////
////                    println("plan execution complete and successful");
////                    // this task is complete
////                    taskComplete(_goalID,
////                        TaskOutcome.PROCESSING_COMPLETE_SUCCESS);
////                }
////                else {
////                    log("step length: " + exeData.getPlanSteps().length);
////
////                    // if plan is not complete
////                    triggerExecution(exeData);
////                    // this task is complete
////                    taskComplete(_goalID,
////                        TaskOutcome.PROCESSING_COMPLETE);
////                }
////
////            }
////            else {
////                log("plan execution not possible, ceasing execution");
////                taskComplete(_goalID,
////                    TaskOutcome.PROCESSING_COMPLETE_FAILURE);
////
////            }
////
////            // if (m_bLogOutput) {
////            // log("Executing:");
////            // log(PlanningUtils.toString(exeData.getPlan()));
////            // }
////        }
////        catch (SubarchitectureProcessException e) {
////            e.printStackTrace();
////        }
//
//    }
//
//    /*
//     * (non-Javadoc)
//     * 
//     * @see cast.architecture.subarchitecture.ManagedProcess#taskRejected(java.lang.String)
//     */
//    @Override
//    protected void taskRejected(String _goalID) {
////        PlanExecutionData lfgd = m_taskMap.remove(_goalID);
////        if (lfgd == null) {
////            println("received unknown goal rejection: " + _goalID);
////        }
//    }
//
//    /*
//     * (non-Javadoc)
//     * 
//     * @see cast.architecture.abstr.WorkingMemoryReaderProcess#workingMemoryChanged(cast.corba.autogen.CAST.WorkingMemoryChange[])
//     */
////    @Override
////    protected void workingMemoryChanged(WorkingMemoryChange[] _wmc) {
////        // always getting linguistic goals when they're added
////
////        for (int i = 0; i < _wmc.length; i++) {
////            try {
////
////                if (_wmc[i].m_type.equals(PlanningOntology.PLAN_TYPE)) {
////                    // this will get called the first time a plan is
////                    // added
////                    // to working memory
////
////                    // only care about one type
////
////                    CASTTypedDataWithID planData = getWorkingMemoryEntry(_wmc[i].m_address.m_id);
////
////                    Plan plan = (Plan) planData.getData();
////                    PlanningProblem initialState = getPlanningProblem(new WorkingMemoryAddress(
////                        plan.m_problemID, m_subarchitectureID));
////
////                    // do the execution monitoring init stuff
////                    String pmID = m_plannerServer.get_PM_ID();
////
////                    PlanExecutionData exeData = new PlanExecutionData(
////                        pmID, _wmc[i].m_address.m_id, plan,
////                        initialState);
////
////                    m_plannerServer.update_planning_memory(exeData
////                        .getPlanMemoryID(), plan.m_plan,
////                        initialState.m_objectList,
////                        initialState.m_factList,
////                        initialState.m_maplGoal,
////                        initialState.m_domainFile);
////
////                    String mapID = groundWaypoints(exeData);
////
////                    // if one of these is true, they both should
////                    if (exeData.allWaypointsSet()) {
////                        // if all grounded, then we're good to go
////                        queuePlan(exeData);
////                    }
////                    else if (mapID != null) {
////                        // if this happens then we have to wait for the
////                        // replies
////                        m_taskMap.put(mapID, exeData);
////                    }
////                    else {
////                        println("unable to ground all waypoints, plan will not be executed");
////                    }
////                }
////                else if (_wmc[i].m_type
////                    .equals(SpatialOntology.SPATIAL_RELATIONSHIP_RESPONSE_TYPE)) {
////
////                    SpatialRelationshipTargetResponse response = getTargetResponse(_wmc[i].m_address);
////
////                    // log("seen response!!!");
////                    //
////                    // for (int j = 0; j < response.m_targets.length;
////                    // j++) {
////                    // log("sweet spot: " + j + " "
////                    // + response.m_targets[j].m_vector.m_x + " "
////                    // + response.m_targets[j].m_vector.m_y + " "
////                    // + response.m_targets[j].m_confidence);
////                    // }
////
////                    String queryID = response.m_queryAddress.m_id;
////
////                    // get the id of the execution data obejct for this
////                    // query
////                    String dataID = m_queryMap.remove(queryID);
////
////                    PlanExecutionData exeData = m_taskMap
////                        .remove(dataID);
////
////                    ArrayList<String> m_wpIDs = m_waypointToQueryMap
////                        .get(queryID);
////
////                    boolean failure = false;
////
////                    // for each repsonse, set the waypoint value
////                    for (int j = 0; j < response.m_targets.length; j++) {
////                        String wpID = m_wpIDs.get(j + 2);
////                        
////                        log(wpID + " response: "
////                            + response.m_targets[j].m_vector.m_x
////                            + " "
////                            + response.m_targets[j].m_vector.m_y
////                            + " confidence ("
////                            + response.m_targets[j].m_confidence
////                            + ")");
////                        
////                        if (response.m_targets[j].m_confidence > m_spatialRelThreshold) {
////                            exeData.setWaypointPosition(wpID,
////                                response.m_targets[j].m_vector);
////
////                            log(wpID + " grounded via spatial query: "
////                                + response.m_targets[j].m_vector.m_x
////                                + " "
////                                + response.m_targets[j].m_vector.m_y
////                                + " confidence ("
////                                + response.m_targets[j].m_confidence
////                                + ")");
////                        }
////                        else {
////                            failure = true;
////                            break;
////                        }
////                    }
////
////                    if (failure) {
////                        println("unable to ground all waypoints after spatial query");
////                    }
////                    else if (exeData.allWaypointsSet()) {
////                        // we're good to go!!!
////                        //
////                        // finally!!!
////                        //
////                        // i can't quite believe it!!!
////                        queuePlan(exeData);
////                    }
////                    else {
////                        // we're still waiting for more spatial queries
////                        m_taskMap.put(dataID, exeData);
////                    }
////
////                }
////                else if (_wmc[i].m_type
////                    .equals(ManipulationOntology.PICK_AND_PLACE_COMMAND_TYPE)) {
////
////                    // check it's from me!
////                    if (m_taskMap.containsKey(_wmc[i].m_address.m_id)) {
////
////                        // the plan step is done!
////                        CASTTypedDataWithID papcData = getWorkingMemoryEntry(_wmc[i].m_address.m_id);
////                        PickAndPlaceCmd papc = (PickAndPlaceCmd) papcData
////                            .getData();
////
////                        // retreive execution data
////                        PlanExecutionData exeData = m_taskMap
////                            .remove(papcData.getId());
////
////                        if (papc.complete == 1) {
////                            log("step execution completed successfully");
////
////                            // advance the monitoring state
////                            advancePlanInPlanMemory(exeData);
////                            // do it twice because every action is in
////                            // fact 2 steps
////                            advancePlanInPlanMemory(exeData);
////
////                            // queue it up for execution
////                            queuePlan(exeData);
////                        }
////                        else {
////                            println("step execution failed: "
////                                + papc.complete);
////                            println("plan execution terminating");
////                        }
////                    }
////
////                }
////
////            }
////            catch (SubarchitectureProcessException e) {
////                e.printStackTrace();
////            }
////        }
////
////        // some dummy methods just to test things!
////        // getCurrentState();
////
////        // processLingusticInput();
////    }
//
//    /*
//     * (non-Javadoc)
//     * 
//     * @see cast.core.components.CASTProcessingComponent#configure(java.util.Properties)
//     */
//    @Override
//    public void configure(Properties _config) {
//        super.configure(_config);
//        if (_config.containsKey("-t")) {
//            m_spatialRelThreshold = Float.parseFloat(_config
//                .getProperty("-t"));
//            log("using threshold value: " + m_spatialRelThreshold);
//        }
//        else {
//            log("using default threshold value: "
//                + m_spatialRelThreshold);
//        }
//
//        m_visualSubarchitectureID = _config.getProperty("-vsa");
//    }
//
//    /**
//     * @param relString
//     * @param relType
//     * @return
//     * @throws PlanningException
//     */
//    public SpatialRelationshipType string2SpatialRelationshipType(
//            String relString) throws PlanningException {
//        if (relString.equals("near")) {
//            return SpatialRelationshipType.SPATIAL_PROXIMAL;
//        }
//        else if (relString.equals("front_of")) {
//            return SpatialRelationshipType.SPATIAL_FRONT;
//        }
//        else if (relString.equals("back_of")) {
//            return SpatialRelationshipType.SPATIAL_BACK;
//        }
//        else if (relString.equals("left_of")) {
//            return SpatialRelationshipType.SPATIAL_LEFT;
//        }
//        else if (relString.equals("right_of")) {
//            return SpatialRelationshipType.SPATIAL_RIGHT;
//        }
//
//        throw new PlanningException(
//            "unknown prep string: " + relString);
//    }

}
