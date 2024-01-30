function [s, sd, sdd, t] = p2p_motion(L,fs,F,fe,A,D,Ts)
%{
P2P_MOTION This function generates the tangential displacement, velocity,
and acceleration profiles for a point to point motion profile
   INPUT(s): 
        L - the travel length/displacement [mm]
        fs - the initial speed [mm/s]
        F - the commanded feedrate [mm/s] 
        fe - the end velocity [mm/s] 
        A - the accleration limit [mm/s^2] 
        D - the deceleration limit (must be negative) [mm/s^2] 
        Ts - the sampling time of the discrete system [s]
   OUTPUT(s):
        s - tangential displacement 
        sd - tang. velocity
        sdd - tang. acceleraton 
        t - cycle time [s]
   
%} 

% Segment time for accelerating and decelerating segments
T1 = (F - fs)/A;
T3 = (fe - F)/D;

% Calculate the time for T2, constant velocity portion
T2 = (1/F)*(L - (1/(2*A) - 1/(2*D))*F^2 - ((fe^2)/(2*D) - (fs^2)/(2*A)));

% if T2 is less than 0, there's not enough time for a constant velocity -
% so we set the time to 0 and recalculate the velocity and the time for the
% accelerating and decellerating portions.
if T2 <= 0 
    T2 = 0;
    F = sqrt(((1/(2*A) - 1/(2*D))^-1)*(L - ((fe^2)/(2*D) - (fs^2)/(2*A))));
    T1 = (F - fs)/A;
    T3 = (fe - F)/D;
end

% Calculate number of time steps
N1 = ceil(T1/Ts);
N2 = ceil(T2/Ts);
N3 = ceil(T3/Ts);

% Adjust the time for rounding
T1 = N1*Ts;
T2 = N2*Ts;
T3 = N3*Ts;

% Adjust the feedrate for rounding
F = (2*L - fs*T1 - fe*T3)/(T1 + 2*T2 + T3);

% Adjust acceleration and deceleration limits for rounding
A = (F - fs)/T1;
D = (fe - F)/T3;

% Create time vector for iteration
t = 0:Ts:(N1+N2+N3)*Ts;
s = zeros(1,length(t));

for i = 1:(N1+N2+N3+1)
    if i >= 1 && i <= N1+1
        s(i) = fs*t(i) + (1/2)*A*(t(i)^2);
        if i == N1+1
            s_1e = s(i);
            s_2e = s(i);
        end
    elseif i > N1+1 && i <= N1+N2+1
        s(i) = s_1e + F*(t(i) - N1*Ts);
        if i == N1+N2+1
            s_2e = s(i);
        end
    else
        s(i) = s_2e + F*(t(i) - (N1+N2)*Ts) + (1/2)*D*(t(i) - (N1+N2)*Ts)^2;
    end
end

% Differentiation of s to get tangential velocity and acceleration
sd = zeros(size(s));
sdd = zeros(size(s));
for i = 1:length(s)
    % velocity
    if i == 1
        sd(i) = 0;
    else
        sd(i) = (s(i) - s(i-1))/Ts;
    end
end

for i = 1:length(s)
    % acceleration
    if i == 1
        sdd(i) = 0;
    else
        sdd(i) = (sd(i) - sd(i-1))/Ts;
    end
end

end

