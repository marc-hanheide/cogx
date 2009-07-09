% $Id$

:- module abduction.

:- interface.

:- import_module list, pair.
:- import_module varset.

:- import_module kb, formula.

% - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -%

:- type marking
	--->	resolved
	;	assumed
	;	unsolved
	.

:- type marked(T) == pair(T, marking).

:- type step
	--->	assume(vsmprop)  % XXX no vars should be here!
	;	resolve_rule(vsmrule, subst)
	;	resolve_fact(vsmprop, subst)
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

%------------------------------------------------------------------------------%

:- implementation.

:- import_module map, set, assoc_list.
:- import_module string.
:- import_module context.

new_proof(Goal, Varset) = proof(vs([Goal], Varset), []).

% - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -%

prove(P0, P, KB) :-
	P0 = proof(vs([G0|Gs], VS0), Ss0),

	% find the leftmost unsolved query
	list.takewhile((pred(_Q-M::in) is semidet :-
		M \= unsolved
			), G0, QDone, QUnsolved),

	(if
		QUnsolved = [A-unsolved|Qs]
	then
		step(QDone, A, Qs, Ss0, VS0, G, Ss, VS, KB),
		P1 = proof(vs([G, G0|Gs], VS), Ss),
		prove(P1, P, KB)
	else
		% proof finished
		P = P0
	).

%------------------------------------------------------------------------------%

:- pred step(list(marked(mprop))::in, mprop::in, list(marked(mprop))::in, list(step)::in, varset::in,
		list(marked(mprop))::out, list(step)::out, varset::out, kb::in) is nondet.

	% TODO: have a proper look at the context modalities

	% assumption
step(QY, A, QN, Steps, VS, QY ++ [A-assumed] ++ QN, [assume(vs(A, VS))|Steps], VS, _KB) :-
%	term.is_ground(A),  % XXX this?
	true.

% - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -%

	% resolution with a fact
step(QY0, m(MA, PA0), QN0, Steps, VS0,
		QY ++ [m(MA, PA)-resolved] ++ QN, [resolve_fact(vs(m(MF, PF), VS), Unifier)|Steps], VS, KB) :-
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
step(QY0, m(MA, PA0), QN0, Steps, VS0,
		QY ++ QInsert ++ QN, [resolve_rule(vs(m(MR, Ante-m(MSucc, PSucc)), VS), Unifier)|Steps], VS, KB) :-

	member(R0, KB^kb_rules),
	R0 = vs(m(MR, Ante0-m(MSucc, PSucc0)), VSR),
	compatible(MR ++ MSucc, MA),

	varset.merge_renaming(VS0, VSR, VS, Renaming),
	m(MR, Ante-m(MSucc, PSucc)) = rename_vars_in_mrule(Renaming, m(MR, Ante0-m(MSucc, PSucc0))),

	unify_formulas(PSucc, PA0, Unifier),

	QInsert = list.map((func(MP0) = MP-unsolved :-
		MP = apply_subst_to_mprop(Unifier, MP0)
			), Ante0) ++ [m(MA, apply_subst_to_formula(Unifier, PA0))-resolved],

	QY = map_fst(apply_subst_to_mprop(Unifier), QY0),
	QN = map_fst(apply_subst_to_mprop(Unifier), QN0).

% - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -%

	% TODO: factoring

% - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -%

:- func map_fst(func(T) = V, list(pair(T, U))) = list(pair(V, U)).

map_fst(Func, LIn) = LOut :-
	LOut = list.map((func(K-V) = apply(Func, K)-V), LIn).
