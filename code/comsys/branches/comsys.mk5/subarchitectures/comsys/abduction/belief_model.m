:- module belief_model.

:- interface.

:- import_module set, string, maybe.
:- import_module formula.
:- import_module stf.
:- import_module stringable.

:- type agent
	--->	robot
	;	human
	.

:- instance stringable(agent).
:- instance parsable(agent).

:- type belief
	--->	private(agent)
	;	attrib(agent, agent)
	;	mutual(set(agent))
	.

:- type foreground
	--->	com
	.

% - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -%

:- type belief_model
	--->	bm(
		k :: set({stf, belief, ground_atomic_formula, maybe(foreground)}),
		t :: set({stf, belief, ground_atomic_formula, maybe(foreground)})
	).

:- func init = belief_model.

%------------------------------------------------------------------------------%

:- implementation.

init = bm(set.init, set.init).

% - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -%

:- instance stringable(agent) where [
	func(to_string/1) is agent_to_string
].

:- instance parsable(agent) where [
	func(from_string/1) is string_to_agent
].

:- pred agent_as_string(agent, string).
:- mode agent_as_string(in, out) is det.
:- mode agent_as_string(out, in) is semidet.

agent_as_string(human, "h").
agent_as_string(robot, "r").

:- func agent_to_string(agent) = string.

agent_to_string(A) = S :- agent_as_string(A, S).

:- func string_to_agent(string::in) = (agent::out) is semidet.

string_to_agent(S) = A :- agent_as_string(A, S).

% - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -%
