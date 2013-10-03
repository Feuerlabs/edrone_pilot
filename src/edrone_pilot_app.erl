-module(edrone_pilot_app).

-behaviour(application).

%% Application callbacks
-export([start/2,
	 start_phase/3,
	 stop/1]).

%% ===================================================================
%% Application callbacks
%% ===================================================================

start(_StartType, _StartArgs) ->
    edrone_pilot_sup:start_link().

start_phase(_, _, _) ->
    ok.

stop(_State) ->
    ok.
