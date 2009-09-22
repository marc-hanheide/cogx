:- module 'TypeConversions_mint'.

:- interface.
:- import_module float, list, bool.
:- import_module ctx_specific, ctx_modality, abduction, formula, belief_model.
:- import_module varset.

:- pred new_with_const_cost_function(mprop(ctx_modality)::in, float::in, with_cost_function(mprop(ctx_modality))::out) is det.

:- pred new_mprop(list(ctx_modality)::in, atomic_formula::in, mprop(ctx_modality)::out, varset::in, varset::out) is det.

:- pred new_atomic_formula(string::in, list(formula.term)::in, atomic_formula::out, varset::in, varset::out) is det.
:- pred new_term(string::in, list(formula.term)::in, formula.term::out, varset::in, varset::out) is det.
:- pred new_var(string::in, formula.term::out, varset::in, varset::out) is det.

:- pred new_varset(varset::out) is det.

% - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -%

:- pred empty_term_list(list(formula.term)::out) is det.
:- pred cons_term_list(formula.term::in, list(formula.term)::in, list(formula.term)::out) is det.

:- pred empty_annots_list(list(with_cost_function(mprop(ctx_modality)))::out) is det.
:- pred cons_annots_list(with_cost_function(mprop(ctx_modality))::in, list(with_cost_function(mprop(ctx_modality)))::in, list(with_cost_function(mprop(ctx_modality)))::out) is det.

:- pred empty_ctx_modality_list(list(ctx_modality)::out) is det.
:- pred cons_ctx_modality_list(ctx_modality::in, list(ctx_modality)::in, list(ctx_modality)::out) is det.

% - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -%

:- pred modality_event(ctx_modality::out) is det.
:- pred modality_info(ctx_modality::out) is det.
:- pred modality_att(ctx_modality::out) is det.
:- pred modality_k(ctx_modality::out) is det.

:- pred impure_print_list_modalities(list(ctx_modality)::in) is det.
:- pred impure_print_modality(ctx_modality::in) is det.

:- pred is_modality_event(ctx_modality::in) is semidet.
:- pred is_modality_info(ctx_modality::in) is semidet.
:- pred is_modality_att(ctx_modality::in) is semidet.
:- pred is_modality_k(ctx_modality::in, belief::out) is semidet.

:- pred belief_private(belief::out) is det.

:- pred is_belief_private(belief::in, string::out) is semidet.
:- pred is_belief_attrib(belief::in, string::out, string::out) is semidet.
:- pred is_belief_mutual(belief::in, list(string)::out) is semidet.

%------------------------------------------------------------------------------%

:- pred dissect_term(varset::in, formula.term::in, bool::out, string::out, list(formula.term)::out) is det.
:- pred dissect_predicate(varset::in, atomic_formula::in, string::out, list(formula.term)::out) is det.
:- pred dissect_mprop(mprop(ctx_modality)::in, list(ctx_modality)::out, atomic_formula::out) is det.
:- pred dissect_proof(ctx::in, proof(ctx_modality)::in, varset::out, list(mprop(ctx_modality))::out,
		float::out) is det.

%------------------------------------------------------------------------------%

:- implementation.
:- import_module io.
:- import_module set, map, list, bool, string, pair, bag, assoc_list.
:- import_module utils.
:- import_module term, term_io, formula.
:- import_module formula_io, formula_ops, ctx_io.
:- import_module solutions, require.
:- import_module varset, costs.
:- import_module modality, stringable, context, costs.
:- import_module belief_model, lf, stf.
:- import_module lf_io.
:- import_module varset.

%------------------------------------------------------------------------------%

:- pragma foreign_export("C", new_mprop(in, in, out, in, out), "new_mprop").

new_mprop(Mod, F, MProp, !VS) :-
	MProp = m(Mod, F).
%	trace [io(!IO)] (print("MProp = " ++ string(MProp), !IO), nl(!IO) ).

:- pragma foreign_export("C", new_atomic_formula(in, in, out, in, out), "new_atomic_formula").

new_atomic_formula(PredSym, Args, F, !VS) :-
	F = p(PredSym, Args).
%	trace [io(!IO)] (print("Atomic formula = " ++ string(F), !IO), nl(!IO) ).

:- pragma foreign_export("C", new_term(in, in, out, in, out), "new_term").

new_term(Functor, Args, T, !VS) :-
	T = t(Functor, Args).
%	trace [io(!IO)] (print("Term = " ++ string(T), !IO), nl(!IO) ).

:- pragma foreign_export("C", new_var(in, out, in, out), "new_var").

new_var(Name, T, !VS) :-
	varset.create_name_var_map(!.VS, VarNames),
	(if map.search(VarNames, Name, Var0)
	then Var = Var0
	else varset.new_named_var(!.VS, Name, Var, !:VS)
	),
	T = v(Var).
%	trace [io(!IO)] (print("Var = " ++ string(T), !IO), nl(!IO) ).

:- pragma foreign_export("C", new_varset(out), "new_varset").

new_varset(varset.init).

:- pragma foreign_export("C", new_with_const_cost_function(in, in, out), "new_with_const_cost_function").

new_with_const_cost_function(MProp, Cost, cf(MProp, const(Cost))).

%------------------------------------------------------------------------------%

:- pragma foreign_export("C", empty_term_list(out), "empty_term_list").
:- pragma foreign_export("C", cons_term_list(in, in, out), "cons_term_list").

empty_term_list([]).
cons_term_list(H, T, [H|T]).

:- pragma foreign_export("C", empty_annots_list(out), "empty_annots_list").
:- pragma foreign_export("C", cons_annots_list(in, in, out), "cons_annots_list").

empty_annots_list([]).
cons_annots_list(H, T, [H|T]).

:- pragma foreign_export("C", empty_ctx_modality_list(out), "empty_ctx_modality_list").
:- pragma foreign_export("C", cons_ctx_modality_list(in, in, out), "cons_ctx_modality_list").

empty_ctx_modality_list([]).
cons_ctx_modality_list(H, T, [H|T]).

%------------------------------------------------------------------------------%

:- pragma foreign_export("C", modality_event(out), "modality_event").
:- pragma foreign_export("C", modality_info(out), "modality_info").
:- pragma foreign_export("C", modality_att(out), "modality_att").
:- pragma foreign_export("C", modality_k(out), "modality_k").

modality_event(e(now)).
modality_info(i).
modality_att(a(com)).
modality_k(k(now, private(human))).

:- pragma foreign_export("C", impure_print_modality(in), "print_modality").
:- pragma foreign_export("C", impure_print_list_modalities(in), "print_list_modalities").

impure_print_modality(Mod) :-
	trace[io(!IO)] (print(string(Mod) ++ "\n", !IO)).

impure_print_list_modalities(Mod) :-
	trace[io(!IO)] (print(string(Mod) ++ "\n", !IO)).

:- pragma foreign_export("C", is_modality_event(in), "is_modality_event").
:- pragma foreign_export("C", is_modality_info(in), "is_modality_info").
:- pragma foreign_export("C", is_modality_att(in), "is_modality_att").
:- pragma foreign_export("C", is_modality_k(in, out), "is_modality_k").

is_modality_event(e(now)).% :- trace[io(!IO)] (print("merc: event\n", !IO)).
is_modality_info(i).% :- trace[io(!IO)] (print("merc: info\n", !IO)).
is_modality_att(a(com)).% :- trace[io(!IO)] (print("merc: att\n", !IO)).
is_modality_k(k(now, Belief), Belief).% :- trace[io(!IO)] (print("merc: k, bel=" ++ string(Belief) ++ "\n", !IO)).

:- pragma foreign_export("C", belief_private(out), "belief_private").

belief_private(private(human)).

:- pragma foreign_export("C", is_belief_private(in, out), "is_belief_private").
:- pragma foreign_export("C", is_belief_attrib(in, out, out), "is_belief_attrib").
:- pragma foreign_export("C", is_belief_mutual(in, out), "is_belief_mutual").

is_belief_private(private(Ag), to_string(Ag)).
is_belief_attrib(attrib(AgA, AgB), to_string(AgA), to_string(AgB)).
is_belief_mutual(mutual(SetAgs), ListAgs) :-
	ListAgs = list.map((func(Ag) = to_string(Ag)), set.to_sorted_list(SetAgs)).

%------------------------------------------------------------------------------%

:- pragma foreign_export("C", dissect_term(in, in, out, out, out), "dissect_term").

dissect_term(VS, v(Var), yes, VarName, []) :-
	varset.lookup_name(VS, Var, "V_", VarName).

dissect_term(_VS, t(Functor, Args), no, Functor, Args).

% - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -%

:- pragma foreign_export("C", dissect_predicate(in, in, out, out), "dissect_predicate").

dissect_predicate(_VS, p(PredSym, Args), PredSym, Args).

% - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -%

:- pragma foreign_export("C", dissect_mprop(in, out, out), "dissect_mprop").

dissect_mprop(m(Mod, Pred), Mod, Pred).
%	trace [io(!IO)] (print("dissecting mprop: " ++ string(m(Mod, Pred)), !IO)),

% - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -%

:- pragma foreign_export("C", dissect_proof(in, in, out, out, out), "dissect_proof").

dissect_proof(Ctx, Proof, VS, LastGoals, Cost) :-
	vs(Qs, VS) = last_goal(Proof),
	LastGoals = list.map((func(Q) = MProp :-
		( Q = proved(MProp)
		; Q = assumed(MProp, _)
		; Q = unsolved(MProp, _)
		; Q = asserted(MTest),
			( MTest = prop(MProp)
			; MTest = impl(_, MProp)
			)
		)), Qs),

	Costs = costs(1.0, 1.0, 0.1),
	Cost = cost(Ctx, Proof, Costs).
%	trace [io(!IO)] ( print(LastGoals, !IO), nl(!IO) ).
