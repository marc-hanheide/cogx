% $Id$

:- module costs.

:- interface.

:- type cost_function
	--->	f(string)  % named cost function
	;	const(float)  % constant cost function
	.

:- func cost_function_to_string(cost_function) = string.

%------------------------------------------------------------------------------%

:- implementation.

:- import_module string.

cost_function_to_string(f(S)) = S.
cost_function_to_string(const(Float)) = string.float_to_string(Float).

