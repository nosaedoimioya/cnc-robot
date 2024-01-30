function [t,q,endIdxMoves] = random_bang_coast_bang(traj_params,Ts,q0)
%{
RANDOM_BANG_COAST_BANG Returns a random bang-coast-bang trajectory for one
axis
   INPUT(s):
       traj_params: matlab struct for the trajectory parameters 
                   {
                   dq_max: maximum position deviation of joint angle 
                   dq_min: minimum position deviation of joint angle 
                   v_max: maximum velocity limit of joint
                   v_min: minimum velocity limit of joint 
                   a_max: maximum acceleration limit of joint
                   t_wait_max: max wait time after a move until next move 
                   t_wait_min: min wait " " 
                   vs: start velocity for each "bang"
                   ve: end velocity for each "
                   end_time: approx. length of the trajectory (in seconds)
                   } 
       Ts: sample time [s]
       q0: initial position
   OUTPUT(s): 
       t: time vector - Nx1
       q: time series bang-coast-bang trajectory - Nx1 
       endIdxMoves: end index of each of the moves - # of moves x 1
                    the number of moves can be different
%}

% max and min of change in joint angle, velocity, and acceleration
dq_max = traj_params.dq_max; 
dq_min = traj_params.dq_min;

v_max = traj_params.v_max;
v_min = traj_params.v_min; 

a_max = traj_params.a_max;
a_min = traj_params.a_min;

% max and min waiting time
t_wait_max = traj_params.t_wait_max;
t_wait_min = traj_params.t_wait_min; 

% start and end velocity of each trajectory
vs = traj_params.vs;
ve = traj_params.ve;

t_end = 0; % trajectory time [s]
q = [];
t = [];
endIdxMoves = [];

while t_end < traj_params.end_time    
    % random motion parameters
    dq = (dq_max-dq_min) * rand(1) + dq_min;
    v = (v_max-v_min) * rand(1) + v_min;
    a = (a_max-a_min) * rand(1) + a_min;

    % generate random motion profile
    [qj,~,~,tj] = p2p_motion(abs(dq),vs,v,ve,a,-a,Ts);
    qj = sign(dq)*qj;

    % random wait time
    t_wait = (t_wait_max-t_wait_min) * rand(1) + t_wait_min;
    tw = tj(end)+Ts:Ts:tj(end)+Ts+t_wait;
    qw = ones(1,length(tw))*qj(end); % keep constant q during wait

    % append to the trajectory
    if t_end == 0
        q = [q; qj'+q0; qw'+q0];
        t = [t; tj'; tw'];
        endIdxMoves = [endIdxMoves; length(qj)+length(tw)];
    else
        q = [q; qj(2:end)'+q(end); qw'+q(end)];
        t = [t; tj(2:end)'+t(end); tw'+t(end)];
        endIdxMoves = [endIdxMoves; length(qj)+length(tw)+endIdxMoves(end)-1];
    end

    t_end = t(end);
end

end