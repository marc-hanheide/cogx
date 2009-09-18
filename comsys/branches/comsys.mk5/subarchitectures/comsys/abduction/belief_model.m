:- module belief_model.

:- interface.

:- import_module set, string, maybe, map.
:- import_module formula, lf.
:- import_module stf, world_model.
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

:- type mbm == map(stf, map(belief, world_model)).

:- type belief_model
	--->	bm(
		k :: set({stf, belief, lf, maybe(foreground)}),
		t :: set({stf, belief, lf, maybe(foreground)})
	).

:- func add_lf_to_mbm(stf, belief, lf, mbm) = mbm.

:- func init = belief_model.

%------------------------------------------------------------------------------%

:- implementation.
:- import_module require.
:- import_module map, world_model, lf_io.

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

add_lf_to_mbm(STF, Bel, LF, MBM0) = MBM :-
	(if map.search(MBM0, STF, BelMap0)
	then BelMap = BelMap0
	else BelMap = map.init
	),
	(if map.search(BelMap, Bel, MFound)
	then M0 = MFound
	else M0 = world_model.init
	),
	(if add_lf(M0, LF, M)
	then
		map.set(BelMap, Bel, M, NewBelMap),
		map.set(MBM0, STF, NewBelMap, MBM)
	else
		error("inconsistent addition of LF \"" ++ lf_to_string(LF) ++ "\" in add_lf_to_mbm")
	).

% - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -%

:- func get_mbm(set({stf, belief, lf, maybe(foreground)})) = mbm.

get_mbm(Set) = MBM :-
	MBM = set.fold((func({STF, Bel, LF, _}, MBM0) = add_lf_to_mbm(STF, Bel, LF, MBM0)), Set, map.init).
