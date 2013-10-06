%%%-------------------------------------------------------------------
%%% @author magnus <magnus@t520>
%%% @copyright (C) 2013, magnus
%%% @doc
%%%
%%% @end
%%% Created : 24 Sep 2013 by magnus <magnus@t520>
%%%-------------------------------------------------------------------
-module(edrone_pilot).

-behaviour(gen_server).

%% API
-export([start_link/0]).

%% gen_server callbacks
-export([init/1, handle_call/3, handle_cast/2, handle_info/2,
	 terminate/2, code_change/3]).

-export([]).

-include("deps/inpevt/include/inpevt.hrl").

-define(SERVER, ?MODULE). 
-define(PITCH_MIN_DEFAULT, 0).
-define(PITCH_MAX_DEFAULT, 1023).

-define(ROLL_MIN_DEFAULT, 0).
-define(ROLL_MAX_DEFAULT, 1023).


-define(YAW_MIN_DEFAULT, 0).
-define(YAW_MAX_DEFAULT, 255).

-define(THROTTLE_MIN_DEFAULT, 0).
-define(THROTTLE_MAX_DEFAULT, 255).

-define(JOYSTICK_DEVICE_DEFAULT, "/dev/input/event17").
-define(DRONE_ADDR_DEFAULT, { "192.168.111.1", 4509 }).


-define(PITCH_CMD,          16#0001).
-define(ROLL_CMD,           16#0002).
-define(YAW_CMD,            16#0003).
-define(THROTTLE_CMD,       16#0004).
-define(PITCH_TRIM_CMD,     16#0005).
-define(ROLL_TRIM_CMD,      16#0006).
-define(YAW_TRIM_CMD,       16#0007).
-define(UPGRADE_CMD,        16#0008).
-define(ALT_LOCK_CMD,       16#0009).
-define(MAX_CMD_VAL,        16#3FF).

-record(st, { 
	  joystick_pid = nil,
	  pitch = 0.0,
	  roll = 0.0,
	  yaw = 0.0,
	  throttle = 0.0,

	  pitch_trim = 0,
	  roll_trim = 0,
	  yaw_trim = 0,

	  pitch_min = ?PITCH_MIN_DEFAULT, 
	  pitch_max = ?PITCH_MAX_DEFAULT, 

	  roll_min = ?ROLL_MIN_DEFAULT, 
	  roll_max = ?ROLL_MAX_DEFAULT, 

	  yaw_min = ?YAW_MIN_DEFAULT, 
	  yaw_max = ?YAW_MAX_DEFAULT, 

	  throttle_min = ?THROTTLE_MIN_DEFAULT, 
	  throttle_max = ?THROTTLE_MAX_DEFAULT, 

	  drone_socket = undefined,
	  drone_address = { undefined, undefined }
	 }).


%%%===================================================================
%%% API
%%%===================================================================

%%--------------------------------------------------------------------
%% @doc
%% Starts the server
%%
%% @spec start_link() -> {ok, Pid} | ignore | {error, Error}
%% @end
%%--------------------------------------------------------------------
start_link() ->
    gen_server:start_link({local, ?SERVER}, ?MODULE, [], []).

%%%===================================================================
%%% gen_server callbacks
%%%===================================================================

%%--------------------------------------------------------------------
%% @private
%% @doc
%% Initializes the server
%%
%% @spec init(Args) -> {ok, St} |
%%                     {ok, St, Timeout} |
%%                     ignore |
%%                     {stop, Reason}
%% @end
%%--------------------------------------------------------------------
init([]) ->

    {ok, { device, JoyPort, _, _, _}} = 
	inpevt:add_device(get_env(joystick_device, ?JOYSTICK_DEVICE_DEFAULT)),

    inpevt:subscribe(JoyPort, self()),
    {DroneIP, DronePort} = get_env(drone_address, ?DRONE_ADDR_DEFAULT),
    {ok, UDPSocket } = gen_udp:open(DronePort), %% Use our target port as the source as well
    {ok, #st { 
       joystick_pid = JoyPort,

       drone_address = {DroneIP, DronePort},
       drone_socket = UDPSocket,
       roll_min = get_env(roll_min, ?ROLL_MIN_DEFAULT), 
       roll_max = get_env(roll_max, ?ROLL_MAX_DEFAULT), 

       pitch_min = get_env(pitch_min, ?PITCH_MIN_DEFAULT), 
       pitch_max = get_env(pitch_max, ?PITCH_MAX_DEFAULT), 

       yaw_min = get_env(yaw_min, ?YAW_MIN_DEFAULT), 
       yaw_max = get_env(yaw_max, ?YAW_MAX_DEFAULT), 

       throttle_min = get_env(throttle_min, ?THROTTLE_MIN_DEFAULT), 
       throttle_max = get_env(throttle_max, ?THROTTLE_MAX_DEFAULT)
      }
    }.

%%--------------------------------------------------------------------
%% @private
%% @doc
%% Handling call messages
%%
%% @spec handle_call(Request, From, St) ->
%%                                   {reply, Reply, St} |
%%                                   {reply, Reply, St, Timeout} |
%%                                   {noreply, St} |
%%                                   {noreply, St, Timeout} |
%%                                   {stop, Reason, Reply, St} |
%%                                   {stop, Reason, St}
%% @end
%--------------------------------------------------------------------

%% flat_trim must be called before we call enable

handle_call(_Request, _From, St) ->
    {reply, ok, St}.
    


%%--------------------------------------------------------------------
%% @private
%% @doc
%% Handling cast messages
%%
%% @spec handle_cast(Msg, St) -> {noreply, St} |
%%                                  {noreply, St, Timeout} |
%%                                  {stop, Reason, St}
%% @end
%%--------------------------------------------------------------------
handle_cast(_Msg, St) ->
    {noreply, St}.

%%--------------------------------------------------------------------
%% @private
%% @doc
%% Handling all non call/cast messages
%%
%% @spec handle_info(Info, St) -> {noreply, St} |
%%                                   {noreply, St, Timeout} |
%%                                   {stop, Reason, St}
%% @end
%%--------------------------------------------------------------------
handle_info(#input_event { type = abs, code_sym = y, value = Val },
	    #st { pitch_min = Min, pitch_max = Max } = St) ->

    NormVal = calc_norm_val(Val, Min, Max),
     io:format("pitch(~p)~n", [ NormVal]),
    send_joystick(pitch, NormVal, St),
    {noreply, St#st { pitch = NormVal }};


handle_info(#input_event { type = abs, code_sym = x, value = Val },
	    #st { roll_min = Min, roll_max = Max } = St) ->

    NormVal = calc_norm_val(Val, Min, Max),
    io:format("              roll(~p)~n", [NormVal]),
    send_joystick(roll, NormVal, St),
    {noreply, St#st { roll = NormVal }};

handle_info(#input_event { type = abs, code_sym = rz, value = Val },
	    #st { yaw_min = Min, yaw_max = Max } = St) ->

    NormVal = calc_norm_val(Val, Min, Max),
    io:format("                           yaw(~p)~n", [ NormVal]),
    send_joystick(yaw, NormVal, St),
    {noreply, St#st { yaw = NormVal }};

handle_info(#input_event { type = abs, code_sym = throttle, value = Val },
	    #st { throttle_min = Min, throttle_max = Max } = St) ->

    %% Max throttle is actually lowest reported value.
    %% Also, range is 0.0..1.0
    NormVal = ?MAX_CMD_VAL - calc_norm_val(Val, Min, Max),
    io:format("                                       throttle(~p)~n", [ NormVal]),
    send_joystick(throttle, NormVal, St),
    {noreply, St#st { throttle = NormVal }};


handle_info(#input_event { type = abs, code_sym = hat0y, value = -1 } , 
	    #st { pitch_trim = PTrim } = St) when PTrim > -512->
    NTrim = PTrim - 1,
    io:format("                                                          pitch_trim(~p)~n", [ NTrim]),
    send_joystick(pitch_trim, calc_norm_val(NTrim, -511, 511), St),
    {noreply, St#st { pitch_trim = NTrim }};

handle_info(#input_event { type = abs, code_sym = hat0y, value = 1 } , 
	    #st { pitch_trim = PTrim } = St) when PTrim < 512->
    NTrim = PTrim + 1,
    io:format("                                                          pitch_trim(~p)~n", [ NTrim]),
    send_joystick(pitch_trim, calc_norm_val(NTrim, -511, 511), St),
    {noreply, St#st { pitch_trim = NTrim }};



handle_info(#input_event { type = abs, code_sym = hat0x, value = -1 } , 
	    #st { roll_trim = PTrim } = St) when PTrim > -512->
    NTrim = PTrim - 1,
    io:format("                                                          roll_trim(~p)~n", [ NTrim]),
    send_joystick(roll_trim, calc_norm_val(NTrim, -511, 511), St),
    {noreply, St#st { roll_trim = NTrim }};

handle_info(#input_event { type = abs, code_sym = hat0x, value = 1 } , 
	    #st { roll_trim = PTrim } = St) when PTrim < 512->
    NTrim = PTrim + 1,
    io:format("                                                          roll_trim(~p)~n", [ NTrim]),
    send_joystick(roll_trim, calc_norm_val(NTrim, -511, 511), St),
    {noreply, St#st { roll_trim = NTrim }};


handle_info(#input_event { type = key, code_sym = pinkie, value = 1 }, 
	    #st { yaw_trim = PTrim } = St)  when PTrim < 512 ->
    NTrim = PTrim + 1,
    io:format("                                                          yaw_trim(~p)~n", [ NTrim]),
    send_joystick(yaw_trim, calc_norm_val(NTrim, -511, 511), St),
    {noreply, St#st { yaw_trim = NTrim }};


handle_info(#input_event { type = key, code_sym = top2, value = 1} , 
	    #st { yaw_trim = PTrim } = St)  when PTrim > -512 ->
    NTrim = PTrim - 1,
    io:format("                                                          yaw_trim(~p)~n", [ NTrim]),
    
    send_joystick(yaw_trim, calc_norm_val(NTrim, -511, 511), St),
    {noreply, St#st { yaw_trim = NTrim }};


handle_info(#input_event { type = key, code_sym = trigger, value = 1}, St) ->
    io:format("Altitude lock: Engaged~n"),
    send_joystick(alt_lock, 1, St),
    {noreply, St};

handle_info(#input_event { type = key, code_sym = trigger, value = 0}, St) ->
    io:format("Altitude lock: Disengaged~n"),
    send_joystick(alt_lock, 0, St),
    {noreply, St};

handle_info(#input_event { type = key, code_sym = thumb, value = 1}, St) ->
    io:format("UPGRADE TIME~n"),
    send_joystick(upgrade, 0, St),
    {noreply, St};



handle_info(#input_event { type = syn }, St) ->
    {noreply, St};


handle_info(_Info, St) ->
%%     io:format("handle_info()??: ~p~n", [ Info ]),
    {noreply, St}.

%%--------------------------------------------------------------------
%% @private
%% @doc
%% This function is called by a gen_server when it is about to
%% terminate. It should be the opposite of Module:init/1 and do any
%% necessary cleaning up. When it returns, the gen_server terminates
%% with Reason. The return value is ignored.
%%
%% @spec terminate(Reason, St) -> void()
%% @end
%%--------------------------------------------------------------------
terminate(_Reason, _St) ->
    ok.

%%--------------------------------------------------------------------
%% @private
%% @doc

%% Convert process st when code is changed
%%
%% @spec code_change(OldVsn, St, Extra) -> {ok, NewSt}
%% @end
%%--------------------------------------------------------------------
%% Enable so that we get a message when a packet is read by uart.
code_change(_OldVsn, St, _Extra) ->
    { ok, St }.
     

get_env(Key, Default) ->
    case application:get_env(?MODULE, Key) of
	undefined -> io:format("get_env(~p): Default: ~p~n", [Key,Default]), Default;
	{ok, Val} -> io:format("get_env(~p): ~p~n", [Key, Val]), Val

    end.
		      
%% Calculate a 0-1023 normalized value based on input
calc_norm_val(Val, Min, Max) ->
    trunc((Val - Min) / (Max - Min) * ?MAX_CMD_VAL).
	    
send_joystick(Key, Val, #st { drone_socket = Socket,
			      drone_address = { Addr, Port } }) ->
    case encode_cmd(Key) of
	error -> {error, unknown_key};
	Field -> gen_udp:send(Socket, Addr, Port, << Field:6/unsigned, Val:10/unsigned >>)
    end.

	    

encode_cmd(pitch) -> ?PITCH_CMD;
encode_cmd(roll) -> ?ROLL_CMD;
encode_cmd(yaw) -> ?YAW_CMD;
encode_cmd(throttle) -> ?THROTTLE_CMD;
encode_cmd(pitch_trim) -> ?PITCH_TRIM_CMD;
encode_cmd(roll_trim) -> ?ROLL_TRIM_CMD;
encode_cmd(yaw_trim) -> ?YAW_TRIM_CMD;
encode_cmd(upgrade) -> ?UPGRADE_CMD;
encode_cmd(alt_lock) -> ?ALT_LOCK_CMD;
encode_cmd(_)-> error.
