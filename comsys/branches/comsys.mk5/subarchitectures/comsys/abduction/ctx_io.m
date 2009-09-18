:- module ctx_io.

:- interface.
:- import_module stringable.

:- import_module belief_model.
:- import_module ctx_modality.

:- instance stringable(ctx_modality).
:- instance term_parsable(ctx_modality).

%------------------------------------------------------------------------------%

:- implementation.
:- import_module list, string, set, term, require.

:- instance stringable(ctx_modality) where [
	func(to_string/1) is ctx_modality_to_string
].

:- instance term_parsable(ctx_modality) where [
	func(from_term/1) is ctx_modality_from_term
].

%------------------------------------------------------------------------------%

:- func l_to_r(pred(A, B), A) = B.
:- mode l_to_r(pred(in, out) is det, in) = (out) is det.

l_to_r(Pred, A) = B :-
	call(Pred, A, B).

:- func r_to_l(pred(A, B), B) = A.
:- mode r_to_l(pred(out, in) is semidet, in) = (out) is semidet.

r_to_l(Pred, B) = A :-
	call(Pred, A, B).

%------------------------------------------------------------------------------%

:- pred foreground_as_string(foreground, string).
:- mode foreground_as_string(in, out) is det.
:- mode foreground_as_string(out, in) is semidet.

foreground_as_string(com, "com").

% - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -%

:- func stf_to_string(stf) = string.
:- func term_to_stf(term.term::in) = (stf::out) is semidet.

stf_to_string(now) = "now".
term_to_stf(functor(atom("now"), [], _)) = now.

% - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -%

:- func ctx_modality_to_string(ctx_modality) = string.
:- func ctx_modality_from_term(term.term::in) = (ctx_modality::out) is semidet.

	% axiom
ctx_modality_to_string(any) = "[]".
ctx_modality_from_term(functor(atom("[]"), [], _)) = any.

	% info
ctx_modality_to_string(i) = "i".
ctx_modality_from_term(functor(atom("i"), [], _)) = i.

	% attention state
ctx_modality_to_string(a(Fgr)) = "a(" ++ FgrStr ++ ")" :- foreground_as_string(Fgr, FgrStr).
ctx_modality_from_term(functor(atom("a"), [
		functor(atom(FgrStr), [], _)
	], _)) = a(Fgr) :- foreground_as_string(Fgr, FgrStr).

	% events
ctx_modality_to_string(e(STF)) = "e(" ++ stf_to_string(STF) ++ ")".
ctx_modality_from_term(functor(atom("e"), [
		STFTerm
	], _)) = e(term_to_stf(STFTerm)).

	% "knows"
ctx_modality_to_string(k(STF, private(A))) = "k(" ++ stf_to_string(STF) ++ "," ++ to_string(A) ++ ")".
ctx_modality_to_string(k(STF, attrib(A, B))) = "k(" ++ stf_to_string(STF) ++ ",[" ++ to_string(A) ++ "]"
		++ to_string(B) ++ ")".
ctx_modality_to_string(k(STF, mutual(AgS))) = "k(" ++ stf_to_string(STF) ++ ",{" ++ AgSStr ++ "})" :-
	AgSStr = string.join_list(",", list.map(to_string, set.to_sorted_list(AgS))).

ctx_modality_from_term(functor(atom("k"), [
		STFTerm,
		BeliefTerm
	], _)) = k(term_to_stf(STFTerm), term_to_belief(BeliefTerm)).

	% tasks
ctx_modality_to_string(t(STF, private(A))) = "t(" ++ stf_to_string(STF) ++ "," ++ to_string(A) ++ ")".
ctx_modality_to_string(t(STF, attrib(A, B))) = "t(" ++ stf_to_string(STF) ++ ",[" ++ to_string(A) ++ "]"
		++ to_string(B) ++ ")".
ctx_modality_to_string(t(STF, mutual(AgS))) = "t(" ++ stf_to_string(STF) ++ ",{" ++ AgSStr ++ "})" :-
	AgSStr = string.join_list(",", list.map(to_string, set.to_sorted_list(AgS))).

ctx_modality_from_term(functor(atom("t"), [
		STFTerm,
		BeliefTerm
	], _)) = t(term_to_stf(STFTerm), term_to_belief(BeliefTerm)).

% - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -%

:- func term_to_belief(term.term::in) = (belief::out) is semidet.

term_to_belief(functor(atom("private"), [functor(atom(AStr), [], _)], _)) = private(from_string(AStr)).
term_to_belief(functor(atom("attrib"), [functor(atom(AStr), [], _), functor(atom(BStr), [], _)], _)) = attrib(from_string(AStr), from_string(BStr)).
term_to_belief(functor(atom("mutual"), ATerms, Context)) = mutual(set.from_list(As)) :-
	list.map((pred(functor(atom(AStr), [], _)::in, from_string(AStr)::out) is semidet), ATerms, As),
	(if As = []
	then
		Context = context(Filename, LineNo),
		error("empty set of mutually-believing agents in " ++ Filename ++ " at line " ++ string.from_int(LineNo))
	else
		true
	).
