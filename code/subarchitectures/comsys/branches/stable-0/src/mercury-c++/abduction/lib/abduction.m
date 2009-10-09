% $Id$

:- module abduction.

:- interface.

:- import_module list, bag.
:- import_module varset.

:- import_module modality.
:- import_module formula, costs, context.

% - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -%

:- type query(M)
	--->	proved(mprop(M))
	;	assumed(mprop(M), cost_function)
	;	unsolved(mprop(M), cost_function)
	;	asserted(mtest(M))
	.

%:- type marked(T) == pair(T, marking).

:- type step(M)
	--->	assume(vscope(mprop(M)), subst, cost_function)
	;	resolve_rule(vscope(mrule(M)), subst)
	;	use_fact(vscope(mprop(M)), subst)
	;	factor(subst, varset)
	.

:- type proof(M)
	--->	proof(
		p_goals :: vscope(list(list(query((M))))),  % in reverse order
		p_steps :: list(step(M))  % in reverse order
	).

:- type costs
	--->	costs(
		fact_cost :: float,
		rule_cost :: float,
		min_assumption_cost :: float
	).

:- type goal(M) == vscope(list(query(M))).

% - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -%

:- func new_proof(list(query(M)), varset) = proof(M) <= modality(M).

:- pred prove(float::in, float::in, proof(M)::in, proof(M)::out, costs::in, C::in) is nondet <= (modality(M), context(C, M)).

:- func last_goal(proof(M)) = vscope(list(query(M))) <= modality(M).

:- func goal_assumptions(goal(M)) = bag(with_cost_function(mgprop(M))) <= modality(M).
:- func goal_assertions(goal(M)) = bag(vscope(mtest(M))) <= modality(M).

:- func step_cost(C, step(M), costs) = float <= (context(C, M), modality(M)).
:- func cost(C, proof(M), costs) = float <= (context(C, M), modality(M)).
%:- func goal_cost(C, goal(M), float) = float <= (context(C, M), modality(M)).

%------------------------------------------------------------------------------%

:- pred interpret(goal(M)::in, C::in, C::out) is det <= (context(C, M), modality(M)).

%------------------------------------------------------------------------------%

:- implementation.

:- import_module require.
:- import_module map, set, assoc_list, pair.
:- import_module string, float.
:- import_module modality.

new_proof(Goal, Varset) = proof(vs([Goal], Varset), []).

% - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -%

goal_assumptions(vs(Qs, _VS)) = As :-
	As = bag.from_list(list.filter_map((func(assumed(MProp, Func)) = AnnotMGProp is semidet :-
		MProp = m(Mod, Prop),
		AnnotMGProp = cf(m(Mod, det_formula_to_ground_formula(Prop)), Func)
			), Qs)).

% - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -%

goal_assertions(vs(Qs, VS)) = As :-
	As = bag.from_list(list.filter_map((func(asserted(MProp)) = vs(MProp, VS) is semidet), Qs)).

% - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -%

last_goal(Proof) = G :-
	(if Proof^p_goals = vs([Last|_Prev], Varset)
	then G = vs(Last, Varset)
	else error("empty proof in last_goal/1")
	).

step_cost(Ctx, assume(VSMProp, _Subst, CostFunction), _Costs) = context.cost(Ctx, CostFunction, VSMProp).
step_cost(_Ctx, use_fact(_, _), Costs) = Costs^fact_cost.
step_cost(_Ctx, resolve_rule(_, _), Costs) = Costs^rule_cost.
step_cost(_Ctx, factor(_, _), _Costs) = 0.0.

cost(Ctx, Proof, Costs) = Cost :-
	list.foldl((pred(Step::in, C0::in, C::out) is det :-
		C = C0 + step_cost(Ctx, Step, Costs)
			), Proof^p_steps, 0.0, Cost).

/*
goal_cost(Ctx, vs(Qs, VS), Costs) = Cost :-
	list.foldl((pred(MProp-Marking::in, C0::in, C::out) is det :-
		( Marking = unsolved(_), C = C0
		; Marking = resolved, C = C0 + CostForUsingFacts
		; Marking = assumed(CostFunction), C = C0 + context.cost(Ctx, CostFunction, vs(MProp, VS))
		; Marking = asserted, C = C0
		)
			), Qs, 0.0, Cost).
*/

% - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -%

:- pred goal_solved(list(query(M))::in) is semidet <= modality(M).

goal_solved(L) :-
	list.all_true((pred(Q::in) is semidet :-
		Q \= unsolved(_, _)
			), L).

% - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -%

prove(CurCost, CostBound, P0, P, Costs, Ctx) :-
	P0 = proof(vs([L0|Ls], VS0), Ss0),
	(if
		goal_solved(L0)
	then
		% check that all assumptions, assertions are ground
		% (because we may have constant weight functions)
		% XXX: check assertions?
		% XXX: check resolved stuff too?
		LAss = list.filter_map((func(Q) = MPr is semidet :-
			Q = assumed(MPr, _)
				), L0),
		all_true((pred(MProp::in) is semidet :-
			ground_formula(MProp^p, _)
				), LAss),
		P = P0
	else
		transform(Step, L0, VS0, L, VS, Ctx),
		P1 = proof(vs([L, L0|Ls], VS), [Step|Ss0]),

		StepCost = step_cost(Ctx, Step, Costs),
		CurCost + StepCost =< CostBound,

		prove(CurCost + StepCost, CostBound, P1, P, Costs, Ctx)
	).

%------------------------------------------------------------------------------%

:- pred segment_proof_state(list(query(M))::in,
		{list(query(M)), with_cost_function(mprop(M)), list(query(M))}::out) is semidet
		<= modality(M).

segment_proof_state(Qs, {QsL, cf(QUnsolved, F), QsR} ) :-
	list.takewhile((pred(Q0::in) is semidet :-
		Q0 \= unsolved(_, _)
			), Qs, QsL, [unsolved(QUnsolved, F) | QsR]).

%------------------------------------------------------------------------------%

:- import_module io, formula_io.

%------------------------------------------------------------------------------%

:- pred transform(step(M)::out,
		list(query(M))::in, varset::in,
		list(query(M))::out, varset::out,
		C::in) is nondet <= (modality(M), context(C, M)).

transform(Step, L0, VS0, L, VS, Ctx) :-
	segment_proof_state(L0, SegL0),
	trace[compile_time(flag("debug")), io(!IO)] ( print(stderr_stream, "[", !IO) ),
	step(Step, SegL0, VS0, L, VS, Ctx),
	trace[compile_time(flag("debug")), io(!IO)] ( print(stderr_stream, "]", !IO) ).

%------------------------------------------------------------------------------%

:- pred step(
		step(M)::out, 

			% input
		{
			list(query(M)),  % unsolved (preceding propositions)
			with_cost_function(mprop(M)),  % proposition under examination + its assump.cost
			list(query(M))  % following propositions
		}::in,
		varset::in,  % variables used in the proof

			% output
		list(query(M))::out,  % resulting goal after performing the step
		varset::out,  % variables used in the goal

		C::in  % knowledge base
	) is nondet <= (modality(M), context(C, M)).


	% assumption
step(assume(vs(m(MQ, PQ), VS), Uni, F),
		{QsL0, cf(m(MQ, PQ0), F), QsR0}, VS0,
		QsL ++ [assumed(m(MQ, PQ), F)] ++ QsR, VS,
		Ctx) :-


	assumable(Ctx, vs(m(MQ, PQ0), VS0), F, vs(m(MA, PA0), VSA), _Cost),
	trace[compile_time(flag("debug")), io(!IO)] ( print(stderr_stream, "a{(" ++ atomic_formula_to_string(VSA, PA0), !IO) ),
	match(compose_list(MQ), compose_list(MA)),
	trace[compile_time(flag("debug")), io(!IO)] ( print(stderr_stream, "~", !IO) ),

	varset.merge_renaming(VS0, VSA, VS, Renaming),
	trace[compile_time(flag("debug")), io(!IO)] ( print(stderr_stream, "!", !IO) ),
	PA = rename_vars_in_formula(Renaming, PA0),

	trace[compile_time(flag("debug")), io(!IO)] ( print(stderr_stream, "@", !IO) ),
	unify_formulas(PQ0, PA, Uni),

	trace[compile_time(flag("debug")), io(!IO)] ( print(stderr_stream, "#", !IO) ),
	PQ = apply_subst_to_formula(Uni, PQ0),
	trace[compile_time(flag("debug")), io(!IO)] ( print(stderr_stream, "$", !IO) ),
	QsL = list.map(apply_subst_to_query(Uni), QsL0),
	trace[compile_time(flag("debug")), io(!IO)] ( print(stderr_stream, "%", !IO) ),
	QsR = list.map(apply_subst_to_query(Uni), QsR0),
	trace[compile_time(flag("debug")), io(!IO)] ( print(stderr_stream, "}", !IO) ).

%	formula.is_ground(Q^p).

% - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -%

	% resolution with a fact
step(use_fact(vs(m(MF, PF), VS), Uni),
		{QsL0, cf(m(MQ, PQ0), _F), QsR0}, VS0,
		QsL ++ [proved(m(MQ, PQ))] ++ QsR, VS,
		Ctx) :-

	fact_found(Ctx, vs(m(MQ, PQ0), VS0), vs(m(MF, PF0), VSF)),
	trace[compile_time(flag("debug")), io(!IO)] ( print(stderr_stream, "f{", !IO) ),
	match(compose_list(MF), compose_list(MQ)),

	varset.merge_renaming(VS0, VSF, VS, Renaming),
	PF = rename_vars_in_formula(Renaming, PF0),

	unify_formulas(PF, PQ0, Uni),

	PQ = apply_subst_to_formula(Uni, PQ0),
	QsL = list.map(apply_subst_to_query(Uni), QsL0),
	QsR = list.map(apply_subst_to_query(Uni), QsR0),
	trace[compile_time(flag("debug")), io(!IO)] ( print(stderr_stream, "}", !IO) ).

% - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -%

	% resolution with a rule
step(resolve_rule(vs(m(MR, Ante-RHead), VS), Uni),
		{QsL0, cf(m(MQ, PQ), _F), QsR0}, VS0,
		QsL ++ QsInsert ++ QsR, VS,
		Ctx) :-

	rule_found(Ctx, vs(m(MQ, PQ), VS0), Rule),
	trace[compile_time(flag("debug")), io(!IO)] ( print(stderr_stream, "r{", !IO) ),
	Rule = vs(m(MR, _-RHead0), VSR),
	( RHead0 = std(m(MH, _))
	; RHead0 = test(prop(m(MH, _)))
	; RHead0 = test(impl(_, m(MH, _)))
	),

	match(compose_list(MR ++ MH), compose_list(MQ)),

	varset.merge_renaming(VS0, VSR, VS, Renaming),
	m(MR, Ante-RHead) = rename_vars_in_mrule(Renaming, Rule^body),

	( RHead = std(m(_, PH))
	; RHead = test(prop(m(_, PH)))
	; RHead = test(impl(_, m(_, PH)))
	),

	unify_formulas(PH, PQ, Uni),

	(
		RHead = std(_),
		QHead = proved(m(MQ, apply_subst_to_formula(Uni, PQ)))
	;
		RHead = test(MTest),
		QHead = asserted(apply_subst_to_mtest(Uni, MTest))
	),

		% XXX have assertion in another rule?
	QsInsert = list.map((func(A) = UniA :-
		( A = std(cf(P, F)), UniA = unsolved(apply_subst_to_mprop(Uni, P), F)
		; A = test(T), UniA = asserted(apply_subst_to_mtest(Uni, T))
		)
			), Ante)
			++ [QHead],

%	QsInsert = list.map((func(cf(P, F)) = apply_subst_to_mprop(Uni, P)-unsolved(F)), Ante)
%			++ [m(MQ, apply_subst_to_formula(Uni, PQ))-resolved],

	QsL = list.map(apply_subst_to_query(Uni), QsL0),
	QsR = list.map(apply_subst_to_query(Uni), QsR0),
	trace[compile_time(flag("debug")), io(!IO)] ( print(stderr_stream, "}", !IO) ).

% - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -%

	% factoring
step(factor(Uni, VS),
		{QsL0, cf(m(MQ, PQ), _F), QsR0}, VS,
		QsL ++ QsR, VS,
		_Ctx) :-

	member(Prev, QsL0),
	( Prev = proved(MProp)
	; Prev = unsolved(MProp, _)
	; Prev = assumed(MProp, _)
	; Prev = asserted(prop(MProp))
	; Prev = asserted(impl(_, MProp))
	),

	trace[compile_time(flag("debug")), io(!IO)] ( print(stderr_stream, "t{", !IO) ),

	MProp = m(MP, PP),
	match(compose_list(MP), compose_list(MQ)),

	unify_formulas(PP, PQ, Uni),

	QsL = list.map(apply_subst_to_query(Uni), QsL0),
	QsR = list.map(apply_subst_to_query(Uni), QsR0),
	trace[compile_time(flag("debug")), io(!IO)] ( print(stderr_stream, "}", !IO) ).

%------------------------------------------------------------------------------%

:- func map_fst(func(T) = V, list(pair(T, U))) = list(pair(V, U)).

map_fst(Func, LIn) = LOut :-
	LOut = list.map((func(K-V) = apply(Func, K)-V), LIn).

% - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -%

:- func apply_subst_to_query(subst, query(M)) = query(M) <= modality(M).

apply_subst_to_query(Subst, unsolved(MProp, F)) = unsolved(apply_subst_to_mprop(Subst, MProp), F).
apply_subst_to_query(Subst, proved(MProp)) = proved(apply_subst_to_mprop(Subst, MProp)).
apply_subst_to_query(Subst, assumed(MProp, F)) = assumed(apply_subst_to_mprop(Subst, MProp), F).
apply_subst_to_query(Subst, asserted(MTest)) = asserted(apply_subst_to_mtest(Subst, MTest)).

%------------------------------------------------------------------------------%

interpret(vs(Qs, _Varset), !Ctx) :-
	% call the effects
	list.foldl((pred(Q::in, !.Ctx::in, !:Ctx::out) is det :-
		(if
			( Q = proved(MProp)
			; Q = assumed(MProp, _)
			; Q = asserted(prop(MProp))
			; Q = asserted(impl(_, MProp))
			),
			MProp = m(Mod, Prop),
			ground_formula(Prop, GProp)
		then
			effect(m(Mod, GProp), !Ctx)
		else
			% this happens only when there is an unsolved or non-ground
			% query in the goal.
			%error("unsolved query in interpret/3")
			true
		)
			), Qs, !Ctx).
