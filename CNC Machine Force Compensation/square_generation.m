% ME 584 - Lab 4 Prelab 
% Andrew Edoimioya
% 4/3/2018
% Trajectory Before Interpolation (TBI)

close all
clear all
clc
%% Trajectory Generation

% Define constants
Fs = 250; %[Hz]
Ts = 1/Fs; % sample time [s]
F = 150*1; % feedrate [mm/s]
A = 5000; % acceleration limit [mm/s^2]
D = -1000; % deceleration limit [mm/s^2]

fs = 0; % start velocity [mm/s]
fe = 0; % end velocity

%Points of interest
P = [0 0; 20 0; 20 20; 0 20; 0 0]*1; %x is in the first column, y in second

%first segment (ramp)
L1 = sqrt(P(2,1)^2 + P(2,2)^2); %sloped line

%second segment (straight line)
L2 = sqrt((P(3,1)-P(2,1))^2 + (P(3,2) - P(2,2))^2);

%third segment (circle)
L3 =  sqrt((P(4,1)-P(3,1))^2 + (P(4,2) - P(3,2))^2);

%fourth segment
L4 =  sqrt((P(5,1)-P(4,1))^2 + (P(5,2) - P(4,2))^2);

% L5 = sqrt((P(6,1)-P(5,1))^2 + (P(6,2)-P(5,2))^2);

maxJerkSpeed = 8; % mm/s

for i = 1:4 % 4 segments
    theta = atan2(P(i+1,2)-P(i,2),P(i+1,1)-P(i,1));
    if i < 4
        theta_next_block = atan2(P(i+2,2)-P(i+1,2),P(i+2,1)-P(i+1,1));
    end
    if i == 1
        fs(i) = 0;
        % calculate ending velocity
        vx1 = F*cos(theta);
        vy1 = F*sin(theta);
        vx2 = F*cos(theta_next_block);
        vy2 = F*sin(theta_next_block);
        curJerk = sqrt((vx1-vx2)^2 + (vy1-vy2)^2);
        
        vMaxFactor = 1;
        if (curJerk > maxJerkSpeed)
            vMaxFactor = maxJerkSpeed/curJerk;
        end
        
        fe(i) = min([F,F*vMaxFactor]);
    elseif i == 4
        fs(i) = fe(i-1);
        fe(i) = 0;
    else
        fs(i) = fe(i-1);
        % calculate ending velocity
        vx1 = F*cos(theta);
        vy1 = F*sin(theta);
        vx2 = F*cos(theta_next_block);
        vy2 = F*sin(theta_next_block);
        curJerk = sqrt((vx1-vx2)^2 + (vy1-vy2)^2);
        
        vMaxFactor = 1;
        if (curJerk > maxJerkSpeed)
            vMaxFactor = maxJerkSpeed/curJerk;
        end
        fe(i) = min([F,F*vMaxFactor]);
    end
end

% [s1, ~, ~, t1] = p2p_motion_lookahead(A,0,L1,fs(1),F,fe(1),Ts);
% [s2, ~, ~, t2] = p2p_motion_lookahead(A,0,L2,fs(2),F,fe(2),Ts);
% [s3, ~, ~, t3] = p2p_motion_lookahead(A,0,L3,fs(3),F,fe(3),Ts);
% [s4, ~, ~, t4] = p2p_motion_lookahead(A,0,L4,fs(4),F,fe(4),Ts);

[s1, ~, ~, t1] = p2p_motion(L1,0,F,0,A,D,Ts);
[s2, ~, ~, t2] = p2p_motion(L2,0,F,0,A,D,Ts);
[s3, ~, ~, t3] = p2p_motion(L3,0,F,0,A,D,Ts);
[s4, ~, ~, t4] = p2p_motion(L4,0,F,0,A,D,Ts);
% [s5, ~, ~, t5] = p2p_motion(L5,0,F,0,A,D,Ts);

%combine
s = [s1 s2(2:end)+s1(end) s3(2:end)+s1(end)+s2(end) s4(2:end)+s1(end)+s2(end)+s3(end)]';

%differentiation of s to get tangential velocity and acceleration
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

% sd = [sd1 sd2+sd1(end) sd3+sd1(end)+sd2(end)]';
% sdd = [sdd1 sdd2+sdd1(end) sdd3+sdd1(end)+sdd2(end)]';
tTBI = [t1 t2(2:end)+t1(end) t3(2:end)+t1(end)+t2(end) t4(2:end)+t1(end)+t2(end)+t3(end)]';
%%
% linear interpolation for 1st segment
x_s = 0;
y_s = 0;

for i = 1:length(s1)
    if i == 1 %to handle the first point
        xRaw1(i) = x_s;
        yRaw1(i) = y_s;
    else
        xRaw1(i) = x_s + ((P(2,1)-P(1,1))/L1)*s1(i);
        yRaw1(i) = y_s + ((P(2,2)-P(1,2))/L1)*s1(i);
    end
end

% update x_s and y_s
x_s = xRaw1(end);
y_s = yRaw1(end);

%linear interpolation for second segment
for i = 1:length(s2)
    xRaw2(i) = x_s + ((P(3,1)-P(2,1))/L2)*s2(i);
    yRaw2(i) = y_s + ((P(3,2)-P(2,2))/L2)*s2(i);
end

% update x_s and y_s
x_s = xRaw2(end);
y_s = yRaw2(end);

%linear interpolation for third segment
for i = 1:length(s3)
    xRaw3(i) = x_s + ((P(4,1)-P(3,1))/L3)*s3(i);
    yRaw3(i) = y_s + ((P(4,2)-P(3,2))/L3)*s3(i);
end

% update x_s and y_s
x_s = xRaw3(end);
y_s = yRaw3(end);

%linear interpolation for fourth segment
for i = 1:length(s4)
    xRaw4(i) = x_s + ((P(5,1)-P(4,1))/L4)*s4(i);
    yRaw4(i) = y_s + ((P(5,2)-P(4,2))/L4)*s4(i);
end

% combine all
xTBI = [xRaw1 xRaw2(2:end) xRaw3(2:end) xRaw4(2:end)]';
yTBI = [yRaw1 yRaw2(2:end) yRaw3(2:end) yRaw4(2:end)]';

%% Plotting
figure(1)
subplot(3,1,1)
plot(tTBI,s)
ylabel('Tang. Displacement [mm]')
grid on
title('Tangential Profiles')

subplot(3,1,2)
plot(tTBI,sd)
grid on
ylabel('Tang. Velocity [mm/s]')

subplot(3,1,3)
plot(tTBI,sdd)
ylabel('Tang. Acceleration [mm/s^2]')
xlabel('Time [s]')
grid on

figure(2)
plot(xTBI, yTBI)
xlabel('X [mm]')
ylabel('Y [mm]')
title('Discretized Trajectory (TBI)')
hold on
axis equal
grid on

% plotfixer

%% Generate derivatives through backward euler

vx_TBI = zeros(length(xTBI),1);
vy_TBI = zeros(length(yTBI),1);
for i = 1:length(xTBI)
    % velocity
    if i == 1
        vx_TBI(i) = 0;
        vy_TBI(i) = 0;
    else
        vx_TBI(i) = (xTBI(i) - xTBI(i-1))/Ts;
        vy_TBI(i) = (yTBI(i) - yTBI(i-1))/Ts;
    end
end

ax_TBI = zeros(length(xTBI),1);
ay_TBI = zeros(length(yTBI),1);
for i = 1:length(xTBI)
    % acceleration
    if i == 1
        ax_TBI(i) = 0;
        ay_TBI(i) = 0;
    else
        ax_TBI(i) = (vx_TBI(i) - vx_TBI(i-1))/Ts;
        ay_TBI(i) = (vy_TBI(i) - vy_TBI(i-1))/Ts;
    end
end

%% Plots
tTBI = (0:length(xTBI)-1)*Ts;
figure(3)
subplot(3,2,1)
plot(tTBI, xTBI)
hold on
grid on
ylabel('X Pos [mm]')
% title('Time Series Position, Velocity and Acceleration')

subplot(3,2,2)
plot(tTBI, yTBI)
hold on
grid on
ylabel('Y Pos [mm]')

subplot(3,2,3)
plot(tTBI, vx_TBI)
hold on
grid on
ylabel('X Vel [mm/s]')

subplot(3,2,4)
plot(tTBI, vy_TBI)
hold on
grid on
ylabel('Y Vel [mm/s]')

subplot(3,2,5)
plot(tTBI, ax_TBI)
hold on
grid on
ylabel('X Acc [mm/s^2]')
xlabel('Time [s]')

subplot(3,2,6)
plot(tTBI, ay_TBI)
ylabel('Y Acc [mm/s^2]')
xlabel('Time [s]')
grid on
hold on

figure(4)
plot(tTBI, sqrt(vx_TBI.^2 + vy_TBI.^2))
hold on
grid on
xlabel('Time [s]')
ylabel('Feedrate [mm/s]')

% plotfixer

save('square_vector_v150_a5mps2','tTBI','xTBI','yTBI')
%The final velocity doesn't actually go to zero because of backward euler.
%Is that okay or should we do something to make it zero?

%% Max and Min commanded Acceleration & Cycle Time
Ax_max = max(ax_TBI); %[mm/s^2]
Ay_max = max(ay_TBI);

Dx_max = min(ax_TBI); %[mm/s^2]
Dy_max = min(ay_TBI);

cyc_time = tTBI(end)*1000; %[ms]

%% Find time delay for overlapping

% At R2, find TA from tang. velocity profile graph
% TA2 = 0.1301 - 0.08; %[s]
% err_R2 = 0.1; %[mm] - guess (older: from TAI)
% theta_R2 = pi/2; % approximate angle at R2 - calculated from traj[rad]
% V1_R2 = 100; %max feed rate into R2 [mm/s]
% 
% tauR2 = sqrt(2*TA2*cos(theta_R2/2)*err_R2/((1-sin(theta_R2/2))*V1_R2)); %time shift [s]
% mR2 = floor(tauR2/Ts);
% 
% % At R3, find TA from tang. velocity profile graph
% TA3 = 0.2601 - 0.21; %[s]
% err_R3 = 0.1; %[mm] - from TAI
% theta_R3 = pi/2; % approximate angle at R3 - calculated from traj [rad]
% V1_R3 = 100; %max feed rate into R3 [mm/s]
% 
% tauR3 = sqrt(2*TA3*cos(theta_R3/2)*err_R3/((1-sin(theta_R3/2))*V1_R3)); %time shift [s]
% mR3 = floor(tauR3/Ts);
% 
% TA4 = 0.3901 - 0.34; %[s]
% err_R4 = 0.1; %[mm] - from TAI
% theta_R4 = pi/2; % approximate angle at R3 - calculated from traj [rad]
% V1_R4 = 100; %max feed rate into R3 [mm/s]
% 
% tauR4 = sqrt(2*TA4*cos(theta_R4/2)*err_R4/((1-sin(theta_R4/2))*V1_R4)); %time shift [s]
% mR4 = floor(tauR4/Ts);
% 
% %% Overlapping
% % X-AXIS
% % At R2
% xRaw1_R2 = xRaw1(length(xRaw1)-mR2+1:length(xRaw1));
% xRaw2_R2 = xRaw2(1:mR2);
% x2 = xRaw1(length(xRaw1)); % last scalar in xRaw1
% xRaw12_C = (xRaw1_R2 + xRaw2_R2) - x2; % segment 1 and 2 overlap combined
% 
% % At R3
% xRaw2_R3 = xRaw2(length(xRaw2)-mR3+1:length(xRaw2));
% xRaw3_R3 = xRaw3(1:mR3);
% x3 = xRaw2(length(xRaw2)); % last scalar in xRaw2
% xRaw23_C = (xRaw2_R3 + xRaw3_R3) - x3;% segment 2 and 3 overlap combined
% 
% % At R4
% xRaw3_R4 = xRaw3(length(xRaw3)-mR4+1:length(xRaw3));
% xRaw4_R4 = xRaw4(1:mR4);
% x4 = xRaw3(length(xRaw3)); % last scalar in xRaw3
% xRaw34_C = (xRaw3_R4 + xRaw4_R4) - x4;% segment 3 and 4 overlap combined
% 
% xTBI_O = [xRaw1(1:length(xRaw1)-mR2), xRaw12_C, xRaw2(mR2+1:length(xRaw2)-mR3),...
%        xRaw23_C, xRaw3(mR3+1:length(xRaw3)-mR4), xRaw34_C, xRaw4(mR4+1:length(xRaw4))]';
% 
% % ------Y-AXIS-------
% % At R2
% yRaw1_R2 = yRaw1(length(yRaw1)-mR2+1:length(yRaw1));
% yRaw2_R2 = yRaw2(1:mR2);
% y2 = yRaw1(length(yRaw1)); % last scalar in xRaw1
% yRaw12_C = (yRaw1_R2 + yRaw2_R2) - y2; % segment 1 and 2 overlap combined
% 
% % At R3
% yRaw2_R3 = yRaw2(length(yRaw2)-mR3+1:length(yRaw2));
% yRaw3_R3 = yRaw3(1:mR3);
% y3 = yRaw2(length(yRaw2)); % last scalar in yRaw2 
% yRaw23_C = (yRaw2_R3 + yRaw3_R3) - y3;% segment 2 and 3 overlap combined
% 
% % At R4
% yRaw3_R4 = yRaw3(length(yRaw3)-mR4+1:length(yRaw3));
% yRaw4_R4 = yRaw4(1:mR4);
% y4 = yRaw3(length(yRaw3)); % last scalar in yRaw3 
% yRaw34_C = (yRaw3_R4 + yRaw4_R4) - y4;% segment 3 and 4 overlap combined
% 
% yTBI_O = [yRaw1(1:length(yRaw1)-mR2), yRaw12_C, yRaw2(mR2+1:length(yRaw2)-mR3),...
%        yRaw23_C, yRaw3(mR3+1:length(yRaw3)-mR4), yRaw34_C, yRaw4(mR4+1:length(yRaw4))]';
%    
% figure(2)
% plot(xTBI_O, yTBI_O, 'r-.')
% legend('Regular TBI', 'TBI with Overlapping')
% axis equal
% plotfixer
% 
% %% Finding the derivatives for overlapped
% 
% vx_TBI_O = zeros(length(xTBI_O),1);
% vy_TBI_O = zeros(length(yTBI_O),1);
% for i = 1:length(xTBI_O)
%     % velocity
%     if i == 1
%         vx_TBI_O(i) = 0;
%         vy_TBI_O(i) = 0;
%     else
%         vx_TBI_O(i) = (xTBI_O(i) - xTBI_O(i-1))/Ts;
%         vy_TBI_O(i) = (yTBI_O(i) - yTBI_O(i-1))/Ts;
%     end
% end
% 
% ax_TBI_O = zeros(length(xTBI_O),1);
% ay_TBI_O = zeros(length(yTBI_O),1);
% for i = 1:length(xTBI_O)
%     % acceleration
%     if i == 1
%         ax_TBI_O(i) = 0;
%         ay_TBI_O(i) = 0;
%     else
%         ax_TBI_O(i) = (vx_TBI_O(i) - vx_TBI_O(i-1))/Ts;
%         ay_TBI_O(i) = (vy_TBI_O(i) - vy_TBI_O(i-1))/Ts;
%     end
% end
% 
% %% Plotting TBI Overlapped
% 
% t_TBI_O = 0:Ts:(Ts*(length(xTBI_O)-1)); %[s]
% t_TBI_O = t_TBI_O';
% 
% figure(3)
% subplot(3,2,1)
% plot(t_TBI_O, xTBI_O, 'r--')
% legend('Regular TBI', 'Overlapped TBI')
% hold off
% 
% subplot(3,2,2)
% plot(t_TBI_O, yTBI_O, 'r--')
% hold off
% 
% subplot(3,2,3)
% plot(t_TBI_O, vx_TBI_O, 'r--')
% hold off
% 
% subplot(3,2,4)
% plot(t_TBI_O, vy_TBI_O, 'r--')
% hold off
% 
% subplot(3,2,5)
% plot(t_TBI_O, ax_TBI_O, 'r--')
% hold off
% 
% subplot(3,2,6)
% plot(t_TBI_O, ay_TBI_O, 'r--')
% hold off
% plotfixer
% 
% % save('square_vector_overlap','t_TBI_O','xTBI_O','yTBI_O')