:- module ctx_io.

:- interface.
:- import_module stringable, term.

:- import_module ctx_impl, ctx_modality.

:- instance stringable(ctx_modality).
:- instance term_parsable(ctx_modality).

%------------------------------------------------------------------------------%

:- implementation.
:- import_module list, string.

:- instance stringable(ctx_modality) where [
	func(to_string/1) is ctx_modality_to_string
].

:- instance term_parsable(ctx_modality) where [
	func(from_term/1) is ctx_modality_from_term
].

% - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -%

:- pred agent_as_string(agent, string).
:- mode agent_as_string(in, out) is det.
:- mode agent_as_string(out, in) is semidet.

agent_as_string(human, "h").
agent_as_string(robot, "r").

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
ctx_modality_to_string(a(com)) = "a(com)".
ctx_modality_from_term(functor(atom("a"), [
		functor(atom("com"), [], _)
	], _)) = a(com).

	% events
ctx_modality_to_string(e(now)) = "e(now)".
ctx_modality_from_term(functor(atom("e"), [
		functor(atom("now"), [], _)
	], _)) = e(now).

	% "knows"
ctx_modality_to_string(k(now, Belief)) = "k(now, " ++ string(Belief) ++ ")".
ctx_modality_from_term(functor(atom("k"), [
		functor(atom("now"), [], _),
		_
	], _)) = k(now, private(robot)).
