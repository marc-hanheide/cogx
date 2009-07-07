% $Id$

:- module abduction.

:- interface.

:- import_module list, pair.
:- import_module term, varset.

:- import_module kb, context, formulae.

% - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -%

:- type marking
	--->	resolved
	;	assumed
	;	unsolved
	.

:- type goal == list(pair(ctxterm, marking)).

:- type step
	--->	assume(ctxterm)
	;	resolve_rule(kbrule, substitution)
	;	resolve_fact(kbfact, substitution)
	.

:- type proof
	--->	proof(
		p_goals :: list(goal),  % in reverse order
		p_steps :: list(step),
		p_varset :: varset
	).

% - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -%

:- func new_proof(goal, varset) = proof.

:- pred prove(proof::in, proof::out, kb::in) is nondet.

%------------------------------------------------------------------------------%

:- implementation.

:- import_module map, set, assoc_list.
:- import_module string.

new_proof(Goal, Varset) = proof([Goal], [], Varset).

% - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -%

prove(P0, P, KB) :-
	P0 = proof([G0|Gs], Ss0, VS0),

	% find the leftmost unsolved query
	list.takewhile((pred(_Q-M::in) is semidet :-
		M \= unsolved
			), G0, QDone, QUnsolved),

	(if
		QUnsolved = [A-unsolved|Qs]
	then
		step(QDone, A, Qs, Ss0, VS0, G, Ss, VS, KB),
		P1 = proof([G, G0|Gs], Ss, VS),
		prove(P1, P, KB)
	else
		% proof finished
		P = P0
	).

%------------------------------------------------------------------------------%

:- pred step(goal::in, ctxterm::in, goal::in, list(step)::in, varset::in,
		goal::out, list(step)::out, varset::out, kb::in) is nondet.

	% TODO: have a proper look at the context modalities

	% assumption
step(QY0, A, QN0, Steps, VS, QY0 ++ [A-assumed] ++ QN0, [assume(A)|Steps], VS, _KB) :-
	true.
%	term.is_ground(A).

% - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -%

	% resolution with a rule
step(QY0, CtxA-A0, QN0, Steps, VS0, QY ++ NewQs ++ [(CtxA-A)-resolved] ++ QN, [resolve_rule(Rx, Unifier)|Steps], VS, KB) :-
	member(R, KB^kb_rules),
	R = rule(CtxR, RAnte, CtxRH-RH, RVS),

	CtxA = CtxR ++ CtxRH,

	varset.merge(VS0, RVS, [RH | list.map(snd, RAnte)], VS, [H|Ante0]),  % XXX: assert equal lengths

	assoc_list.from_corresponding_lists(list.map(fst, RAnte), Ante0, Ante),

	Rx = rule(CtxR, Ante, CtxRH-H, VS),
	unify_term(H, A0, init, Unifier),

	NewQ0 = apply_substitution_to_list(Ante0, Unifier),
	assoc_list.from_corresponding_lists(list.map(fst, RAnte), NewQ0, NewQ1),
	NewQs = list.map((func(T) = T-unsolved), NewQ1),

	A = apply_rec_substitution(A0, Unifier),
	QY = map_fst(applysubst(Unifier), QY0),
	QN = map_fst(applysubst(Unifier), QN0).

% - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -%

	% resolution with a fact
step(QY0, CtxA-A0, QN0, Steps, VS0, QY ++ [(CtxA-A)-resolved] ++ QN, [resolve_fact(Fx, Unifier)|Steps], VS, KB) :-
	member(F, KB^kb_facts),
	F = fact(CtxF-FTerm, FVS),

	compatible(CtxF ++ CtxA, CtxA),

	varset.merge(VS0, FVS, [FTerm], VS, [Term]),  % XXX: this is det, not semidet

	Fx = fact(CtxF-Term, VS),
	unify_term(Term, A0, init, Unifier),

	A = apply_rec_substitution(A0, Unifier),
	QY = map_fst(applysubst(Unifier), QY0),
	QN = map_fst(applysubst(Unifier), QN0).

% - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -%

	% TODO: factoring

/*
step(QY0, A0, QN0, Steps, VS0, QY ++ [A-resolved] ++ QN, [resolve_fact(Fx, Unifier)|Steps], VS, KB) :-

	member(F, KB^kb_facts),
	F = fact(FTerm, FVS),

	varset.merge(VS0, FVS, [FTerm], VS, [Term]),  % XXX this is det, not semidet

	Fx = fact(FTerm, VS),
	unify_term(Term, A0, init, Unifier),
	A = apply_rec_substitution(A0, Unifier),
	QY = map_fst(applysubst(Unifier), QY0),
	QN = map_fst(applysubst(Unifier), QN0).
*/
% - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -%

:- func applysubst(substitution, ctxterm) = ctxterm.

applysubst(Subst, Ctx-Term) = Ctx-apply_rec_substitution(Term, Subst).

:- func map_fst(func(T) = V, list(pair(T, U))) = list(pair(V, U)).

map_fst(Func, LIn) = LOut :-
	LOut = list.map((func(K-V) = apply(Func, K)-V), LIn).
