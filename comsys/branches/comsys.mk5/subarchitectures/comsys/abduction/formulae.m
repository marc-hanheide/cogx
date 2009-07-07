% $Id$

:- module formulae.

:- interface.

:- import_module term, varset, pair, list, string.
:- import_module context, kb.

:- type varterm == pair(term, varset).

:- func string_to_term_varset(string) = pair(term, varset).
:- pred string_to_term_varset(string::in, term::out, varset::out) is det.

% - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -%

:- type modal(M, T)
	--->	m(M, T).

:- type ctxterm == pair(list(ctx), term).
:- type ctxvarterm == pair(ctxterm, varset).

:- type kbfact
	--->	fact(
		f_fact :: ctxterm,
		f_varset :: varset
	).

:- type kbrule
	--->	rule(
		r_ctx :: list(ctx),
		r_antecedent :: list(ctxterm),
		r_conclusion :: ctxterm,
		r_varset :: varset
	).

% - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -%

:- pred term_varset_to_kbitem(term::in, varset::in, kbitem::out) is det.

% - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -%

:- pred string_to_ctx(string, ctx).
:- mode string_to_ctx(in, out) is semidet.
:- mode string_to_ctx(out, in) is det.
:- func ctx_to_string(list(ctx)) = string.

:- func term_to_string(varset, term) = string.
:- func ctxvarterm_to_string(ctxvarterm) = string.
:- func ctxterm_to_string(varset, ctxterm) = string.
:- func varterm_to_string(varterm) = string.

%------------------------------------------------------------------------------%

:- implementation.

:- import_module require.
:- import_module term_io, parser.


string_to_term_varset(String) = T-VS :-
	string_to_term_varset(String, T, VS).

string_to_term_varset(String, T, VS) :-
	read_term_from_string("", String, _, RT),
	(if RT = term(VS0, T0)
	then T = T0, VS = VS0, generic_term(T)
	else error("Something went wrong while processing term \"" ++ String ++ "\".")
	).

% - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -%

string_to_ctx("a0", att(this)).
string_to_ctx("a1", att(next)).
string_to_ctx("i0", info(this)).
string_to_ctx("i1", info(next)).
string_to_ctx("e0", evt(this)).
string_to_ctx("e1", evt(next)).
string_to_ctx("axiom", axiom).

ctx_to_string(LCtx) = Str :-
	Str = string.join_list(":", list.map((func(C) = S :- string_to_ctx(S, C)), LCtx)).

:- pred term_to_ctx(term::in, list(ctx)::out) is det.

term_to_ctx(Term, LCtx) :-
	(if
		Term = functor(atom(Str), [], _),
		string_to_ctx(Str, Ctx0)
	then
		LCtx = [Ctx0]
	else
		LCtx = []
	).

:- pred term_varset_to_ctx_term(term::in, varset::in, list(ctx)::out, term::out) is det.

term_varset_to_ctx_term(Term, Varset, Ctx, Term0) :-
	(if Term = functor(atom(":"), [TCtx, TMain], _)
	then
		term_to_ctx(TCtx, Ctx),
		Term0 = TMain
	else
		Ctx = [],
		Term0 = Term
	).

term_varset_to_kbitem(Term0, Varset, Rule) :-
	term_varset_to_ctx_term(Term0, Varset, Ctx, Term),
	(if Term = functor(atom("->"), [TBody, TResult], _)
	then Rule = r(rule(Ctx, term_to_conj_list_of_ctxterms(TBody), []-TResult, Varset))
	else Rule = f(fact(Ctx-Term, Varset))
	).

:- func term_to_conj_list_of_ctxterms(term) = list(ctxterm).

term_to_conj_list_of_ctxterms(Term) = LTerms :-
	(if Term = functor(atom(","), [T, Ts], _)
	then LTerms = [[]-T] ++ term_to_conj_list_of_ctxterms(Ts)
	else LTerms = [[]-Term]
	).

% - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -%

:- func term_const_to_string(term.const) = string.

term_const_to_string(atom(Str)) = Str.
term_const_to_string(integer(Int)) = string.from_int(Int).
term_const_to_string(string(Str0)) = "\"" ++ Str0 ++ "\"".
term_const_to_string(float(_)) = "#.#".
term_const_to_string(implementation_defined(Str)) = "`" ++ Str ++ "`".

% - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -%

ctxvarterm_to_string((Ctx-Term)-Varset) = CtxStr ++ ":" ++ term_to_string(Varset, Term) :-
	CtxStr = string.join_list(":", list.map((func(C) = S :-
		string_to_ctx(S, C)
			), Ctx)).

ctxterm_to_string(Varset, CtxTerm) = ctxvarterm_to_string(CtxTerm-Varset).

varterm_to_string(Term-Varset) = term_to_string(Varset, Term).

term_to_string(Varset, variable(V, _)) = varset.lookup_name(Varset, V).
term_to_string(_, functor(Const, [], _)) = term_const_to_string(Const).
term_to_string(Varset, functor(Const, [H|T], _)) = term_const_to_string(Const) ++ "(" ++ ArgStr ++ ")" :-
	ArgStr = string.join_list(", ", list.map(term_to_string(Varset), [H|T])).

