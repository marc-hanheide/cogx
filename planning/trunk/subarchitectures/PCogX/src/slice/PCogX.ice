/*
 *
 * SLICE (acronym :: Specification Language for Ice).
 *
 * NOTE: Some of the names in this SLICE file will appear odd. I wrote
 * it assuming the underscore character '_' could be used in names,
 * then later discovered it couldn't. What a stupid restriction. I was
 * also thrown by the fact that capitalisation of names in SLICE
 * definitions is ignored.
 */


#ifndef HWD_ICE
#define HWD_ICE

module PCogX
{
    /* A "designator" is a term that.. wait for it.. designates.. in
     * the sense that it makes an assignment of one "object" to some
     * other "object". What follows is a definition of a "SLICE
     * optional class member". For the purposes of planning, a
     * designator should specify exactly what planning subsystem to
     * use for \subarchitecture{planning} and interactions with
     * \subarchitecture{execution} (and perhaps
     * \subarchitecture{motivation}).
     *
     * NOTE : A SLICE convention is to have optional sequence types
     * for class members that are.. well.. optional. We suppose that
     * there can be an optional \type{string} that determines a valid
     * implementation of a procedure call.*/
    ["c++:type:vector<string>"] sequence<string> OptionalMemberDesignator;


    /* When a domain specification element (declaration of planning
     * types (see e.g., \procedure{postActionDefinition}), action
     * description (see e.g., \procedure{postTypes}), etc) is posted
     * to the planning subarchitecture, then the domain -- string
     * representation -- to which the element should be associated is
     * specified. We suppose this can be optional because by default
     * the system can suppose that the element can be associated with
     * the current domain of discourse. */
    ["c++:type:vector<string>"] sequence<string> OptionalMemberDomainDescriptor;


    /* When a subarchitecture asks for a planner, it can specify the
     * properties of that planner. This is of course optional, and
     * where a string specifying some desirable properties of the
     * planner is not given, we suppose the client would like an
     * optimal classical deterministic STRIPS like planning system --
     * more-or-less. In more detail, we assume the following string:
     *
     * (:requirements :typing :strips :equality :fluents)
     *
     * Charles, you are annoying me and being non-specific, moreover
     * you are speaking about yourself in the third person again, and
     * that can only mean you're bored. Okay, so how tall is Danny
     * DeVito? 5 ft or 1.52 m tall.. according to
     * \url{powerset.com}. Awesome chap though Danny...
     *
     * If you are specifying a POMDP, then you will want to post
     * requirements on the planner according to the following example.
     * 
     * ---------------
     * BEGIN EXAMPLE : (this is taken from a domain description of a version
     * of tire-world that exhibits partial observability)
     * ---------------
     *
     * (:requirements ;; Every string that poses requirements on the
     *                ;; planner should begin like this...
     *
     *  :partial-observability ;; Not in IPC-5 tireworld
     *  :fluents ;; Not in IPC-5  tireworld
     *  :universal-effects  ;; Not in IPC-5 tireworld
     *  :conditional-effects  ;; Not in IPC-5 tireworld
     * 
     *  :typing 
     *  :strips 
     *  :equality 
     *  :probabilistic-effects
     *
     * ) ;; Every string that poses requirements on the planner should
     *   ;; end with a closing brace.
     *
     * -----------
     * END EXAMPLE
     * -----------
     *
     * Finally, the following comprises a list of PDDL-based
     * properties that we might support at some time or another over
     * the course of the CogX project. I have appended the substring
     * "**" where support is guaranteed.
     *
     * - :partial-observability        **
     * - :universal-effects            **
     * - :equality                     **
     * - :probabilistic-effects        **
     * - :strips                       **
     * - :adl
     * - :typing                       **
     * - :disjunctive-preconditions
     * - :existential-preconditions
     * - :universal-preconditions
     * - :quantified-preconditions
     * - :conditional-effects          **
     * - :fluents                      **
     * - :time
     * - :preferences
     * - :constraints
     *
     */
    ["c++:type:std::vector<std::string>"] sequence<string> OptionalMemberPlannerDescriptor;


    /* Make a planning component answer to a new name.
     *
     * I.e., the way cast works, there can only be "one" component
     * loaded at any given time. But, of course sometimes you want to
     * instantiate a component multiple times at runtime. When you
     * make a procedure call, the implementer is designated. Here, we
     * suppose some implementers may be able to have a single
     * procedure two-times designated.. or even more times
     * designated. For the planning architecture,
     * \procedure{obtainPlanner} simply asks an existing planning
     * component to listen to calls at a different new
     * designation. That new designation is returned to the
     * \procedure{obtainPlanner} caller. From then on, it should be
     * able to interact with the planner as if it were a new
     * object. Posting domains and problems, reading off plans,
     * etc.*/
    class distinctPlanner
    {   
        ["c++:type:std::string"] string additionalDesignationIsAnArgument;
        
        OptionalMemberDesignator optionalMemberDesignatorIsAnArgument;
    };
    
    /* Obtain a planner with ID \return{identityOfCreatedPlanner}. The
     * latter is a valid \SCLICE::type{OptionalMemberDesignator}. If
     * there is more than one planner factory operating under CAST,
     * then a call is unstable unless
     * \argument{optionalMemberDesignatorIsAnArgument} is specified --
     * or course, in this case each factory must have a unique
     * $Designator$.*/
    class obtainPlanner
    {
        /* String identifier of the planning domain.*/
        ["c++:type:std::string"] string identityOfCreatedPlannerIsAReturn;

        /* Identifier of the planner factory, just in case there is
         * more than one factory operating. */
        OptionalMemberDesignator optionalMemberDesignatorIsAnArgument;
        
        /* Description of the features that can occur in the planning
         * domain at hand.*/
        OptionalMemberPlannerDescriptor optionalMemberPlannerDescriptorIsAnArgument;
    };

    /* From what file should the planner parse the domain description.*/
    class postFileNameForDomainDescription
    {
        /* String identifier of the planning domain.*/
        ["c++:type:std::string"] string fileNameForDomainDescriptionIsAnArgument;

        OptionalMemberDesignator optionalMemberDesignatorIsAnArgument;
    };

    /* From what file should the planner parse the problem description.*/
    class postFileNameForProblemDescription
    {
        /* String identifier of the planning domain.*/
        ["c++:type:std::string"] string fileNameForProblemDescriptionIsAnArgument;

        OptionalMemberDesignator optionalMemberDesignatorIsAnArgument;
    };
    
    /* Typically a planner will associated an integer ID with every
     * proposition (\type{string}). This procedure allows a client of
     * the planner to obtain that association via
     * \return{propositionIDs}.*/
    ["c++:type:std::map<int, std::string>"] dictionary<int, string> PropositionIDs;
    class readPropositionIdentifiers
    {
        // dictionary<int, string>
        PropositionIDs propositionIDsIsAReturn;
        
        OptionalMemberDesignator optionalMemberDesignatorIsAnArgument;
        OptionalMemberDomainDescriptor optionalMemberDomainDescriptorIsAnArgument;
    };
        
    /* Members of the planning community often talk about STRIPS
     * actions (or operators). The STRIPS acronym stands for:
     * "Stanford Research Institute Problem Solver".
     *
     * The action specified by a \procedure{postSTRIPSAction}
     * executes instantaneously in parallel with other actions (STRIPS
     * or otherwise) that are not conflicting. The action specified
     * can only be executed in a state where propositions
     * \argument{precondition} -- i.e., here we suppose a string
     * representation -- are true. In the state resulting from an
     * execution of the specified action, propositions in
     * \argument{addList} are true, and those in
     * \argument{deleteList} are false.*/
    ["c++:type:std::vector<std::string>"] sequence<string> AddList;
    ["c++:type:std::vector<std::string>"] sequence<string> DeleteList;
    ["c++:type:std::vector<std::string>"] sequence<string> Precondition;
    class postSTRIPSAction
    {
        AddList addListIsAnArgument;
        DeleteList deleteListIsAnArgument;
        Precondition preconditionIsAnArgument;
        
        OptionalMemberDesignator optionalMemberDesignatorIsAnArgument;
        OptionalMemberDomainDescriptor optionalMemberDomainDescriptorIsAnArgument;
    };


    /* \argument{actionDefinition} must be a valid PDDL specification
     * of a domain action. The result \return{success} is true
     * provided the parser is happy with
     * \argument{actionDefinition}. Where an error occurs during the
     * parse, if something useful might be said about the error wrt
     * diagnosing what went wrong, that information is posted to
     * \result{parserComplaint}.*/
    class postActionDefinition
    {
        /* NOTE :: I did not find that SLICE supports the C++-98
         * std::rope. Otherwise I perhaps would have used that.*/
        ["c++:type:std::string"] string actionDefinitionIsAnArgument;
        
        /* Was the post successful.*/
        bool successIsAReturn;

        /* Parsing failed, thus \return{success} is false on return
         * from this procedure. This string _may_ contain some useful
         * information about the reason the parse failed.*/
        ["c++:type:std::string"] string parserComplaintIsAReturn;
        
        /* i.e., targeted planner might have a string name. */
        OptionalMemberDesignator optionalMemberDesignatorIsAnArgument;
        OptionalMemberDomainDescriptor optionalMemberDomainDescriptorIsAnArgument;
    };


    /* PDDL specification fragment relating domain types to the
     * planner. */
    class postTypes
    {
        /* For the purposes of PDDL and planning more general, we
         * suppose a type is a unary-predicate that occurs in a
         * "subtype-of" relation with other types.*/
        ["c++:type:std::string"] string typesDefinitionIsAnArgument;
        
        /* Was the post successful.*/
        bool successIsAReturn;

        /* Parsing failed, thus \return{success} is false on return
         * from this procedure. This string _may_ contain some useful
         * information about the reason the parse failed.*/
        ["c++:type:std::string"] string ParserComplaintIsAReturn;
        
        OptionalMemberDesignator optionalMemberDesignatorIsAnArgument;
        OptionalMemberDomainDescriptor optionalMemberDomainDescriptorIsAnArgument;
    };

    /* Declare that types (see \procedure{postTypes}) in the sequence
     * \argument{XX} are subtypes of those listed in
     * \argument{YYIsAnArgument}.*/
    ["c++:type:std::vector<std::string>"] sequence<string> XX;
    ["c++:type:std::vector<std::string>"] sequence<string> YY;
    class postXXsubtypeofYY
    {
        XX XXIsAnArgument;
        YY YYIsAnArgument;
        OptionalMemberDesignator optionalMemberDesignatorIsAnArgument;
        OptionalMemberDomainDescriptor optionalMemberDomainDescriptorIsAnArgument;
    };
};


#endif
