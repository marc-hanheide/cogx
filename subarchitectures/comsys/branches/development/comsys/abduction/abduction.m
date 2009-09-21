% $Id$

:- module abduction.

:- interface.

:- import_module list, pair, set.
:- import_module varset.

:- import_module kb, formula, costs, context.

% - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -%

:- type marking
	--->	resolved
	;	assumed
	;	unsolved(cost_function)
	.

:- type marked(T) == pair(T, marking).

:- type step
	--->	assume(vsmprop, cost_function)  % XXX no vars should be here!
	;	resolve_rule(vsmrule, subst)
	;	use_fact(vsmprop, subst)
	.

:- type proof
	--->	proof(
			% think about variable scopes here
		p_goals :: vscope(list(list(marked(mprop)))),  % in reverse order
		p_steps :: list(step)  % in reverse order
	).

% - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -%

:- func new_proof(list(marked(mprop)), varset) = proof.

:- pred prove(proof::in, proof::out, kb::in) is nondet.

:- func last_goal(proof) = vscope(list(marked(mprop))).

:- func assumed(proof) = set(vsmprop).
:- func cost(d_ctx, proof) = float.

%------------------------------------------------------------------------------%

:- implementation.

:- import_module require.
:- import_module map, set, assoc_list.
:- import_module string, float.
:- import_module context.

new_proof(Goal, Varset) = proof(vs([Goal], Varset), []).

% - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -%

assumed(Proof) = As :-
	vs(Qs, Varset) = last_goal(Proof),
	As = set.from_list(list.filter_map((func(MProp-assumed) = VSMProp is semidet :-
		VSMProp = vs(MProp, Varset)
			), Qs)).

% - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -%

last_goal(Proof) = G :-
	(if Proof^p_goals = vs([Last|_Prev], Varset)
	then G = vs(Last, Varset)
	else error("empty proof")
	).

cost(DCtx, Proof) = Cost :-
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
			C = C0 + 1.0
		;
			Step = resolve_rule(_, _),
			C = C0
		)
			), Proof^p_steps, 0.0, Cost).

% - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -%

prove(P0, P, KB) :-
	P0 = proof(vs([G0|Gs], VS0), Ss0),

	% find the leftmost unsolved query
	list.takewhile((pred(_Q-M::in) is semidet :-
		M \= unsolved(_)
			), G0, QDone, QUnsolved),

	(if
		QUnsolved = [A-unsolved(F)|Qs]
	then
		step(QDone, A, F, Qs, Ss0, VS0, G, Ss, VS, KB),
		P1 = proof(vs([G, G0|Gs], VS), Ss),
		prove(P1, P, KB)
	else
		% proof finished
		P = P0
	).

%------------------------------------------------------------------------------%

:- pred step(
			% input
		list(marked(mprop))::in,  % unsolved (preceding propositions)
		mprop::in,  % unsolved proposition A under examination
		cost_function::in,  % assumption cost function for A
		list(marked(mprop))::in,  % propositions that follow A in the goal
		list(step)::in,  % steps so far (reversed)
		varset::in,  % variables used in the proof

			% output
		list(marked(mprop))::out,  % resulting goal after performing the step
		list(step)::out,  % steps used to devise the goal
		varset::out,  % variables used in the goal

		kb::in  % knowledge base
	) is nondet.


	% assumption
step(QY, m(M, P), F, QN, Steps, VS,
		QY ++ [m(M, P)-assumed] ++ QN, [assume(vs(m(M, P), VS), F)|Steps], VS, _KB) :-
%	formula.is_ground(P),  % XXX this?
	true.

% - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -%

	% resolution with a fact
step(QY0, m(MA, PA0), _F, QN0, Steps, VS0,
		QY ++ [m(MA, PA)-resolved] ++ QN, [use_fact(vs(m(MF, PF), VS), Unifier)|Steps], VS, KB) :-
	member(vs(m(MF, PF0), VSF), KB^kb_facts),
	compatible(MF, MA),

	varset.merge_renaming(VS0, VSF, VS, Renaming),
	PF = rename_vars_in_formula(Renaming, PF0),

	unify_formulas(PF, PA0, Unifier),

	PA = apply_subst_to_formula(Unifier, PA0),
	QY = map_fst(apply_subst_to_mprop(Unifier), QY0),
	QN = map_fst(apply_subst_to_mprop(Unifier), QN0).

% - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -%

	% resolution with a rule
step(QY0, m(MA, PA0), _F, QN0, Steps, VS0,
		QY ++ QInsert ++ QN, [resolve_rule(vs(Axiom-m(MR, Ante-m(MSucc, PSucc)), VS), Unifier)|Steps], VS, KB) :-

	member(R0, KB^kb_rules),
	R0 = vs(Axiom-m(MR, Ante0-m(MSucc, PSucc0)), VSR),
	compatible(MR ++ MSucc, MA),

	varset.merge_renaming(VS0, VSR, VS, Renaming),
	Axiom-m(MR, Ante-m(MSucc, PSucc)) = rename_vars_in_mrule(Renaming, Axiom-m(MR, Ante0-m(MSucc, PSucc0))),

	unify_formulas(PSucc, PA0, Unifier),

	QInsert = list.map((func(cf(MP0, F)) = MP-unsolved(F) :-
		MP = apply_subst_to_mprop(Unifier, MP0)
			), Ante) ++ [m(MA, apply_subst_to_formula(Unifier, PA0))-resolved],

	QY = map_fst(apply_subst_to_mprop(Unifier), QY0),
	QN = map_fst(apply_subst_to_mprop(Unifier), QN0).

% - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -%

	% factoring
step(QY0, m(MA, PA), _F, QN0, Steps, VS,
		QY ++ QN, Steps, VS, _KB) :-
	member(m(MPrec, PPrec)-_, QY0),
	compatible(MPrec, MA),

	unify_formulas(PPrec, PA, Unifier),

	QY = map_fst(apply_subst_to_mprop(Unifier), QY0),
	QN = map_fst(apply_subst_to_mprop(Unifier), QN0).

% - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -%

:- func map_fst(func(T) = V, list(pair(T, U))) = list(pair(V, U)).

map_fst(Func, LIn) = LOut :-
	LOut = list.map((func(K-V) = apply(Func, K)-V), LIn).
