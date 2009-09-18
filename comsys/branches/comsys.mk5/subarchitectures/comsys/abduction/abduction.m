% $Id$

:- module abduction.

:- interface.

:- import_module list, pair, bag.
:- import_module varset.

:- import_module formula, costs, context.

% - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -%

:- type marking
	--->	resolved
	;	assumed
	;	unsolved(cost_function)
	;	asserted
	.

:- type marked(T) == pair(T, marking).

:- type step
	--->	assume(vsmprop, cost_function)  % XXX no vars should be here!
	;	resolve_rule(vsmrule, subst)
	;	use_fact(vsmprop, subst)
	;	factor(subst, varset)
	.

:- type proof
	--->	proof(
			% think about variable scopes here
		p_goals :: vscope(list(list(marked(mprop)))),  % in reverse order
		p_steps :: list(step)  % in reverse order
	).

% - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -%

:- func new_proof(list(marked(mprop)), varset) = proof.

:- pred prove(proof::in, proof::out, ctx::in) is nondet.

:- func last_goal(proof) = vscope(list(marked(mprop))).

:- func assumptions(proof) = bag(vsmprop).
:- func cost(d_ctx, proof, float) = float.

%------------------------------------------------------------------------------%

:- implementation.

:- import_module require.
:- import_module map, set, assoc_list.
:- import_module string, float.
:- import_module ctx_modality.
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

cost(DCtx, Proof, CostForUsingFacts) = Cost :-
	list.foldl((pred(Step::in, C0::in, C::out) is det :-
		(
			Step = assume(VSMProp, CostFunction),
			(
				CostFunction = const(Const),
				C = C0 + Const
			;
				CostFunction = f(FName),
				C = C0 + context.apply_cost_function(DCtx, FName, VSMProp)
			)
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

prove(P0, P, Ctx) :-
	P0 = proof(vs([L0|Ls], VS0), Ss0),

	(if
		%QUnsolved = [A-unsolved(F)|Qs]
		transform(Step, L0, VS0, L, VS, Ctx)
	then
		P1 = proof(vs([L, L0|Ls], VS), [Step|Ss0]),
		prove(P1, P, Ctx)
	else
		% proof finished
		P = P0
	).

%------------------------------------------------------------------------------%

:- pred segment_proof_state(list(marked(mprop))::in,
		{list(marked(mprop)), with_cost_function(mprop), list(marked(mprop))}::out) is semidet.

segment_proof_state(Qs, {QsL, cf(QUnsolved, F), QsR} ) :-
	list.takewhile((pred(_-Label::in) is semidet :-
		Label \= unsolved(_)
			), Qs, QsL, [QUnsolved-unsolved(F) | QsR]).

%------------------------------------------------------------------------------%

:- pred transform(step::out,
		list(marked(mprop))::in, varset::in,
		list(marked(mprop))::out, varset::out,
		ctx::in) is nondet.

transform(Step, L0, VS0, L, VS, Ctx) :-
	segment_proof_state(L0, SegL0),
	step(Step, SegL0, VS0, L, VS, Ctx).

%------------------------------------------------------------------------------%

:- pred step(
		step::out, 

			% input
		{
			list(marked(mprop)),  % unsolved (preceding propositions)
			with_cost_function(mprop),  % proposition under examination + its assump.cost
			list(marked(mprop))  % following propositions
		}::in,
		varset::in,  % variables used in the proof

			% output
		list(marked(mprop))::out,  % resulting goal after performing the step
		varset::out,  % variables used in the goal

		ctx::in  % knowledge base
	) is nondet.


	% assumption
step(assume(vs(m(MQ, PQ), VS), F),
		{QsL0, cf(m(MQ, PQ0), F), QsR0}, VS0,
		QsL ++ [m(MQ, PQ)-assumed] ++ QsR, VS,
		Ctx) :-

	assumable(Ctx, vs(m(MA, PA0), VSA)),
	match(compose_list(MQ), compose_list(MA)),

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
