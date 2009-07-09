% $Id$

:- module test_formulae.

:- interface.

:- import_module io.

:- pred main(io::di, io::uo) is det.

%------------------------------------------------------------------------------%

:- implementation.

:- import_module string, pair, list, map.
:- import_module varset.
:- import_module formula.

main(!IO) :-
	test_mprop_parse("p(x).", !IO),
	test_mprop_parse("p(A).", !IO),
	test_mprop_parse("a.", !IO),
	test_mprop_parse("a(b(c)).", !IO),
	test_mprop_parse("e0:p(x).", !IO),
	test_mprop_parse("i0:a0:p(x).", !IO),

	nl(!IO),

	test_mrule_parse("a -> b.", !IO),
	test_mrule_parse("a, b, c -> d.", !IO),
	test_mrule_parse("i0:(e0:first(x), e0:second(x) -> a0:head(x)).", !IO),

	nl(!IO),

	test_unify("p(X).", "p(Y).", !IO),
	test_unify("p(a, b).", "p(X, b).", !IO),
	test_unify("p(X, b).", "p(a, b).", !IO),
	test_unify("p(x).", "p(y).", !IO),
	test_unify("r(x).", "p(x).", !IO),
	test_unify("p(a, b).", "p(a).", !IO),
	test_unify("p(X, a).", "p(Y, Y).", !IO),
	test_unify("p(X, X).", "p(a, Y).", !IO),
	test_unify("p(X, Y).", "p(Z, Z).", !IO),
	test_unify("p(X).", "p(p(y)).", !IO).
	
% - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -%
/*
:- pred test_term_parse(string::in, io::di, io::uo) is det.

test_term_parse(S, !IO) :-
	string_to_term_varset(S, T, V),
	format("* `%s':\n  parsed=%s\n\n", [s(S), s(string(T-V))], !IO).
*/
% - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -%

:- pred test_mprop_parse(string::in, io::di, io::uo) is det.

test_mprop_parse(S, !IO) :-
	format("(mprop) \"%s\" ... ", [s(S)], !IO),
	(if string_as_vsmprop(S, P)
	then print(P, !IO), nl(!IO), string_as_vsmprop(SNew, P),
			format("        \"%s\"\n", [s(SNew)], !IO)
	else print("fail", !IO), nl(!IO)
	).

% - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -%

:- pred test_mrule_parse(string::in, io::di, io::uo) is det.

test_mrule_parse(S, !IO) :-
	format("(mrule) \"%s\" ... ", [s(S)], !IO),
	(if string_as_vsmrule(S, R)
	then print(R, !IO), nl(!IO), string_as_vsmrule(SNew, R),
			format("        \"%s\"\n", [s(SNew)], !IO)
	else print("fail", !IO), nl(!IO)
	).

% - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -%

:- pred test_unify(string::in, string::in, io::di, io::uo) is det.

test_unify(A, B, !IO) :-
	vs(MPA, VSA) = det_string_to_vsmprop(A),
	vs(MPB0, VSB) = det_string_to_vsmprop(B),

	varset.merge_renaming(VSA, VSB, VS, Renaming),
	MPB = rename_vars_in_mprop(Renaming, MPB0),

	format("(unify) \"%s\" == \"%s\": ", [s(vsmprop_to_string(vs(MPA, VS))),
			s(vsmprop_to_string(vs(MPB, VS)))], !IO),

	MPA = m(_, PA),
	MPB = m(_, PB),

	(if
		unify_formulas(PA, PB, Unifier)
	then
		print(subst_to_string(VS, Unifier), !IO),
		print(" --> ", !IO),
		format("\"%s\", \"%s\"\n",
			[s(vsmprop_to_string(vs(apply_subst_to_mprop(Unifier, MPA), VS))),
			s(vsmprop_to_string(vs(apply_subst_to_mprop(Unifier, MPB), VS)))], !IO)
	else
		print("not unifiable.\n", !IO)
	).

% - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -%

:- func subst_to_string(varset, subst) = string.

subst_to_string(Varset, Subst) = Str :-
	L = map.to_assoc_list(Subst),
	L0 = list.map((func(Var-Value) = S :-
		S = varset.lookup_name(Varset, Var) ++ "=" ++ formula_term_to_string(Varset, Value)), L),
	Str = string.join_list(", ", L0).
