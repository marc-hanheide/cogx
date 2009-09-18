:- module belief_model.

:- interface.

:- import_module set, string, int, maybe, map, int.
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
		k :: map(int, {stf, belief, lf}),
		fg :: set(int),
		next_index :: int
	).

:- func init = belief_model.

:- pred add_lf_to_k(stf::in, belief::in, lf::in, int::out, belief_model::in, belief_model::out) is det.
:- pred foreground(int::in, belief_model::in, belief_model::out) is det.

:- func get_mbm(set({stf, belief, lf})) = mbm.

:- pred k_fact(belief_model::in, stf::out, belief::out, lf::out) is nondet.

%------------------------------------------------------------------------------%

:- implementation.
:- import_module require.
:- import_module map, world_model, lf_io.

init = bm(map.init, set.init, 1).

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

	% error if multiple K's found for the V
:- func reverse_search(map(K, V)::in, V::in) = (K::out) is semidet.

reverse_search(Map, V) = K :-
	map.search(reverse_map(Map), V, Ks),
	(Ks = set.init -> fail ;
	(singleton_set(Ks, K0) -> K = K0 ;
	error("multiple keys in func reverse_search/2")
	)).

% - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -%

add_lf_to_k(STF, Bel, LF, Index, !BM) :-
	(if OldIndex = reverse_search(!.BM^k, {STF, Bel, LF})
	then
		% already there
		Index = OldIndex
	else
		Index = !.BM^next_index,
		!:BM = !.BM^k := map.set(!.BM^k, Index, {STF, Bel, LF}),
		!:BM = !.BM^next_index := !.BM^next_index + 1
	).

% - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -%

foreground(Index, !BM) :-
	(if map.search(!.BM^k, Index, _)
	then !:BM = !.BM^fg := set.insert(!.BM^fg, Index)
	else error("foregrounding a non-existent belief/task")
	).

% - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -%

:- func add_lf_to_mbm(stf, belief, lf, mbm) = mbm.

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

get_mbm(Set) = MBM :-
	MBM = set.fold((func({STF, Bel, LF}, MBM0) = add_lf_to_mbm(STF, Bel, LF, MBM0)), Set, map.init).

% - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -%

k_fact(BM, STF, Bel, LF) :-
	MBM = get_mbm(set.from_list(map.values(BM^k))),
	k_fact0(MBM, STF, Bel, LF).

:- pred k_fact0(mbm::in, stf::out, belief::out, lf::out) is nondet.

k_fact0(MBM, STF, Bel, LF) :-
	map.member(MBM, STF, BelMap),
	map.member(BelMap, Bel0, M),
	set.member(LF, lfs(M)),
	(
		Bel0 = mutual(AgS),
		set.member(Ag, AgS),
		Bel = private(Ag)
	;
		Bel = Bel0
	).
