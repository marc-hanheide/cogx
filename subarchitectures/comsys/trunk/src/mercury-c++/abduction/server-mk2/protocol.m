:- module protocol.

:- interface.

:- type modality
	--->	k
	.

:- type request
	--->	init_ctx
	;	load_file(string)
	;	clear_rules
	;	clear_facts
	;	clear_facts_by_modality(modality)
	;	clear_assumables
	;	add_fact
	;	add_assumable
	;	prove
	;	get_best_proof
	.

:- pred is_request(request::in) is det.

%------------------------------------------------------------------------------%

:- implementation.

is_request(_).
