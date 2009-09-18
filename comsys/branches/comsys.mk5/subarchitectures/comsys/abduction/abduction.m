% $Id$

:- module abduction.

:- interface.

:- import_module list, pair, bag.
:- import_module varset.

:- import_module modality.
:- import_module formula, costs, context.

% - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -%

:- type marking
	--->	resolved
	;	assumed
	;	unsolved(cost_function)
	;	asserted
	.

:- type marked(T) == pair(T, marking).

:- type step(M)
	--->	assume(vscope(mprop(M)), cost_function)  % XXX no vars should be here!
	;	resolve_rule(vscope(mrule(M)), subst)
	;	use_fact(vscope(mprop(M)), subst)
	;	factor(subst, varset)
	.

:- type proof(M)
	--->	proof(
			% think about variable scopes here
		p_goals :: vscope(list(list(marked(mprop(M))))),  % in reverse order
		p_steps :: list(step(M))  % in reverse order
	).

% - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -%

:- func new_proof(list(marked(mprop(M))), varset) = proof(M) <= modality(M).

:- pred prove(proof(M)::in, proof(M)::out, C::in) is nondet <= (modality(M), context(C, M)).

:- func last_goal(proof(M)) = vscope(list(marked(mprop(M)))) <= modality(M).

:- func assumptions(proof(M)) = bag(vscope(mprop(M))) <= modality(M).
:- func cost(C, proof(M), float) = float <= (context(C, M), modality(M)).

%------------------------------------------------------------------------------%

:- implementation.

:- import_module require.
:- import_module map, set, assoc_list.
:- import_module string, float.
:- import_module modality.

new_proof(Goal, Varset) = proof(vs([Goal], Varset), []).

% - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -%

assumptions(Proof) = As :-
	vs(Qs, Varset) = last_goal(Proof),
	As = bag.from_list(list.filter_map((func(MProp-assumed) = VSMProp is semidet :-
		VSMProp = vs(MProp, Varset)
			), Qs)).

% - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -%

last_goal(Proof) = G :-
	(if Proof^p_goals = vs([Last|_Prev], Varset)
	then G = vs(Last, Varset)
	else error("empty proof")
	).

cost(Ctx, Proof, CostForUsingFacts) = Cost :-
	list.foldl((pred(Step::in, C0::in, C::out) is det :-
		(
			Step = assume(VSMProp, CostFunction),
			C = C0 + context.cost(Ctx, CostFunction, VSMProp)
		;
			Step = use_fact(_, _),
			C = C0 + CostForUsingFacts
		;
			Step = resolve_rule(_, _),
			C = C0
		;
			Step = factor(_, _),
			C = C0
		)
			), Proof^p_steps, 0.0, Cost).

% - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -%

:- pred goal_solved(list(marked(mprop(M)))::in) is semidet <= modality(M).

goal_solved(L) :-
	list.all_true((pred(_-Marking::in) is semidet :-
		Marking \= unsolved(_)
			), L).

% - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -%

prove(P0, P, Ctx) :-
	P0 = proof(vs([L0|Ls], VS0), Ss0),
	(if
		goal_solved(L0)
	then
		% check that all assumptions, assertions are ground
		% (because we may have constant weight functions)
		% XXX: check resolved stuff too?
		LAss = list.filter((pred(_MProp-Marking::in) is semidet :-
			( Marking = asserted
			; Marking = assumed
			)
				), L0),
		all_true((pred(MProp-_Marking::in) is semidet :-
			formula.is_ground(MProp^p)
				), LAss),
		P = P0
	else
		transform(Step, L0, VS0, L, VS, Ctx),
		P1 = proof(vs([L, L0|Ls], VS), [Step|Ss0]),
		prove(P1, P, Ctx)
	).

%------------------------------------------------------------------------------%

:- pred segment_proof_state(list(marked(mprop(M)))::in,
		{list(marked(mprop(M))), with_cost_function(mprop(M)), list(marked(mprop(M)))}::out) is semidet
		<= modality(M).

segment_proof_state(Qs, {QsL, cf(QUnsolved, F), QsR} ) :-
	list.takewhile((pred(_-Label::in) is semidet :-
		Label \= unsolved(_)
			), Qs, QsL, [QUnsolved-unsolved(F) | QsR]).

%------------------------------------------------------------------------------%

:- pred transform(step(M)::out,
		list(marked(mprop(M)))::in, varset::in,
		list(marked(mprop(M)))::out, varset::out,
		C::in) is nondet <= (modality(M), context(C, M)).

transform(Step, L0, VS0, L, VS, Ctx) :-
	segment_proof_state(L0, SegL0),
	step(Step, SegL0, VS0, L, VS, Ctx).

%------------------------------------------------------------------------------%

:- pred step(
		step(M)::out, 

			% input
		{
			list(marked(mprop(M))),  % unsolved (preceding propositions)
			with_cost_function(mprop(M)),  % proposition under examination + its assump.cost
			list(marked(mprop(M)))  % following propositions
		}::in,
		varset::in,  % variables used in the proof

			% output
		list(marked(mprop(M)))::out,  % resulting goal after performing the step
		varset::out,  % variables used in the goal

		C::in  % knowledge base
	) is nondet <= (modality(M), context(C, M)).


	% assumption
step(assume(vs(m(MQ, PQ), VS), F),
		{QsL0, cf(m(MQ, PQ0), F), QsR0}, VS0,
		QsL ++ [m(MQ, PQ)-assumed] ++ QsR, VS,
		Ctx) :-

	assumable(Ctx, vs(m(MQ, PQ0), VS0), F, vs(m(MA, PA0), VSA), _Cost),
%	match(compose_list(MQ), compose_list(MA)),

	varset.merge_renaming(VS0, VSA, VS, Renaming),
	PA = rename_vars_in_formula(Renaming, PA0),

	unify_formulas(PQ0, PA, Uni),

	PQ = apply_subst_to_formula(Uni, PQ0),
	QsL = map_fst(apply_subst_to_mprop(Uni), QsL0),
	QsR = map_fst(apply_subst_to_mprop(Uni), QsR0).

%	formula.is_ground(Q^p).

% - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -%

	% resolution with a fact
step(use_fact(vs(m(MF, PF), VS), Uni),
		{QsL0, cf(m(MQ, PQ0), _F), QsR0}, VS0,
		QsL ++ [m(MQ, PQ)-resolved] ++ QsR, VS,
		Ctx) :-

	fact(Ctx, vs(m(MF, PF0), VSF)),
	match(compose_list(MF), compose_list(MQ)),

	varset.merge_renaming(VS0, VSF, VS, Renaming),
	PF = rename_vars_in_formula(Renaming, PF0),

	unify_formulas(PF, PQ0, Uni),

	PQ = apply_subst_to_formula(Uni, PQ0),
	QsL = map_fst(apply_subst_to_mprop(Uni), QsL0),
	QsR = map_fst(apply_subst_to_mprop(Uni), QsR0).

% - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -%

	% resolution with a rule
step(resolve_rule(vs(m(MR, Ante-m(MH, PH)), VS), Uni),
		{QsL0, cf(m(MQ, PQ), _F), QsR0}, VS0,
		QsL ++ QsInsert ++ QsR, VS,
		Ctx) :-

	vrule(Ctx, Rule),
	Rule = vs(m(MR, _-m(MH, _)), VSR),
	match(compose_list(MR ++ MH), compose_list(MQ)),

	varset.merge_renaming(VS0, VSR, VS, Renaming),
	m(MR, Ante-m(MH, PH)) = rename_vars_in_mrule(Renaming, Rule^body),

	unify_formulas(PH, PQ, Uni),

	QsInsert = list.map((func(cf(P, F)) = apply_subst_to_mprop(Uni, P)-unsolved(F)), Ante)
			++ [m(MQ, apply_subst_to_formula(Uni, PQ))-resolved],

	QsL = map_fst(apply_subst_to_mprop(Uni), QsL0),
	QsR = map_fst(apply_subst_to_mprop(Uni), QsR0).

% - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -%

	% factoring
step(factor(Uni, VS),
		{QsL0, cf(m(MQ, PQ), _F), QsR0}, VS,
		QsL ++ QsR, VS,
		_Ctx) :-
	member(m(MP, PP)-_, QsL0),
	match(compose_list(MP), compose_list(MQ)),

	unify_formulas(PP, PQ, Uni),

	QsL = map_fst(apply_subst_to_mprop(Uni), QsL0),
	QsR = map_fst(apply_subst_to_mprop(Uni), QsR0).

% - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -%

	% assertion
%step(QY0, _, 

%------------------------------------------------------------------------------%

:- func map_fst(func(T) = V, list(pair(T, U))) = list(pair(V, U)).

map_fst(Func, LIn) = LOut :-
	LOut = list.map((func(K-V) = apply(Func, K)-V), LIn).
