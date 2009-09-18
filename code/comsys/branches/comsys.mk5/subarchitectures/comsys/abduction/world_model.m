:- module world_model.

:- interface.
:- import_module string, int, map, set, pair.
:- import_module lf.

:- type world_id(Index)
	--->	this
	;	u(int)  % unnamed
	;	i(Index)
	.

:- type world_model(Index, Sort, Rel)
	--->	wm(
		names :: map(Index, Sort),  % sort
		reach :: set({Rel, world_id(Index), world_id(Index)}),  % reachability
		props :: map(world_id(Index), set(proposition)),  % valid propositions
		next_unnamed :: int  % lowest unused index for unnamed worlds
	).

:- type world_model == world_model(string, string, string).

:- func init = world_model.

:- pred add_lf(lf::in, world_model::in, world_model::out) is semidet.

%------------------------------------------------------------------------------%

:- implementation.
:- import_module list.


init = wm(map.init, set.init, map.init, 0).

% - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -%

:- pred new_unnamed_world(int::out, world_model::in, world_model::out) is det.

new_unnamed_world(Id, WM0, WM) :-
	Id = WM0^next_unnamed,
	WM = WM0^next_unnamed := Id + 1.

% - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -%

:- pred rename_merge_world(world_id(string)::in, world_id(string)::in, world_model::in,
		world_model::out) is det.

rename_merge_world(Old, New, WM0, WM) :-
	WM1 = WM0^reach := set.map((func({Rel, OldIdA, OldIdB}) = {Rel, NewIdA, NewIdB} :-
		(OldIdA = Old -> NewIdA = New ; NewIdA = OldIdA),
		(OldIdB = Old -> NewIdB = New ; NewIdB = OldIdB)
			), WM0^reach),

	AssocL = list.map((func(OldId-Props) = NewId-Props :-
		(OldId = Old -> NewId = New ; NewId = OldId)
			), map.to_assoc_list(WM1^props)),

	list.foldl((pred(Name-AddProps::in, Ps0::in, Ps::out) is det :-
		(if map.search(Ps0, Name, OldProps)
		then NewProps = set.union(OldProps, AddProps)
		else NewProps = AddProps
		),
		Ps = map.set(Ps0, Name, NewProps)
			), AssocL, WM1^props, PropsRenamed),

	WM = WM1^props := PropsRenamed.

%------------------------------------------------------------------------------%

add_lf(LF, !WM) :-
	add_lf0(this, _, LF, !WM).

% - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -%

:- pred add_lf0(world_id(string)::in, world_id(string)::out, lf::in,
		world_model::in, world_model::out) is semidet.

add_lf0(Cur, Cur, at(of_sort(WName, Sort), LF), WM0, WM) :-
	% add the referenced world
	(if map.search(WM0^names, WName, OldSort)
	then
		% we've already been there
		OldSort = Sort,  % conflict? => fail
		WM1 = WM0
	else
		% it's a new one
		WM1 = WM0^names := map.det_insert(WM0^names, WName, Sort)
	),
	add_lf0(i(WName), _, LF, WM1, WM).

add_lf0(Cur, i(WName), i(of_sort(WName, Sort)), WM0, WM) :-
	(
	 	Cur = this,
		fail  % should we perhaps allow this?
	;
		Cur = i(WName),
		map.search(WM0^names, WName, Sort),
		WM = WM0
	;
		Cur = u(Num),
		(if map.search(WM0^names, WName, OldSort)
		then OldSort = Sort, WM1 = WM0
		else WM1 = WM0^names := map.det_insert(WM0^names, WName, Sort)
		),
		rename_merge_world(u(Num), i(WName), WM1, WM)
	).

add_lf0(Cur, Cur, r(Rel, LF), WM0, WM) :-
	new_unnamed_world(Num, WM0, WM1),
	WM2 = WM1^reach := set.insert(WM0^reach, {Rel, Cur, u(Num)}),
	add_lf0(u(Num), _, LF, WM2, WM).

add_lf0(Cur, Cur, p(Prop), WM0, WM) :-
	(if OldProps = map.search(WM0^props, Cur)
	then NewProps = set.insert(OldProps, Prop)
	else NewProps = set.make_singleton_set(Prop)
	),
	WM = WM0^props := map.set(WM0^props, Cur, NewProps).
	
add_lf0(Cur0, Cur, and(LF1, LF2), WM0, WM) :-
	add_lf0(Cur0, Cur1, LF1, WM0, WM1),
	add_lf0(Cur1, Cur, LF2, WM1, WM).
