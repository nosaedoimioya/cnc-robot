close all
clear all
clc
%% Edition of this code: 
% This code version aims to use a 3rd or 5th order TF to fit the base FRFs
% based on the robot pose (V and R). Note that the frequency range for fitting 
% can be vary based on the pose.

% 5th order TF is used to fit the shoulder. I am analyzing the first and second
% poles from the base and shoulder FRFs.
% 
% Furtheremore, this code select 4 different lines for all zetas and wn we
% have. The ideia here is use a fixed R and change V. After that, we will
% extract the parameters and use in the LTV input-shapping script. 
% 
%% Edited 05/30/2023
%
% For this example, we are NOT considering the data from the BASE JOINT!!!
% This script is a modification of Fitting_base_and_shoulder_FRF_With_different_TF_order_V4
% In this code, we will extract the zeros and poles of the 5th order SHOULDER TF.
%% ZEROS: TF has 4 zeros: 2 complex conjugate zeros and 2 real ones
%% POLES: TF has 5 poles: 2 complex conjugate pairs of poles and 1 real pole
%% Edited 06/12/2023
% I will add the elbow FRFs.
%% Edited 06/28/2023
% In this vertion I will compare some TF of the system for the base and
% shoulder.
% End-effector position/Rotory encoder - [rad]/[rad] - H
% End-effector position/desired joint trajectory - [rad]/[rad] - H_full
% Rotory encoder/desired joint trajectory - [rad]/[rad] - H_joint


%% Loading files and FRFs - base

dir_date_str = '26-Jun-2023'; % Directory where the FRFs are
date_str     = '26-Jun-2023';
S            = dir(fullfile('D:\OneDrive - Umich\PhD\System ID 5 kg\FRFs\FRFs',dir_date_str,'*.mat')); % Saving the information of all files .m present in the folder 
N            = {S.name}; % Getting only the name of the files
X            = contains(N,'FRF_base_'); % Selecting just the FRFs of the base using the name of the file
X_idx        = find(X);
I            = cell(length(X_idx),1); % creating the variable to save file names
for i = 1:length(X_idx)
    I{i}     = N{X_idx(i)}; % Saving the files names of the Base FRFs
end

% load the first file for getting the right shape
filename     = fullfile('D:\OneDrive - Umich\PhD\System ID 5 kg\FRFs\FRFs',dir_date_str,I{1});
load(filename);

H_B          = zeros(length(H_Y1),length(X_idx)); 
H_B_full     = zeros(length(H_Y1_full),length(X_idx)); 
H_B_joint    = zeros(length(H_Y1_joint),length(X_idx)); 

% Working values
% z: 2, p: 3, cutoff: 40Hz

% for fitting the FRF - base
freq_range_fit = horzcat(0.001, freq_range); % We need to put a very low frequency to be the starting
                                             % point with high weight.
H_B_fit        = zeros(length(freq_range_fit),length(X_idx));
% Section the frequency range of interest to fit the fist pole 

%% Shoulder LSQ fitting 
num_order_shoulder = 4; % this we should change
den_order_shoulder = 5;

%% Elbow LSQ fitting 
num_order_elbow = 3; % this we should change
den_order_elbow = 4;

joint_positions_base    = zeros(length(X_idx),6); % for storing joint positions
V_rad_base              = zeros(length(X_idx),1); % for storing V angles
R_m_base                = zeros(length(X_idx),1); % for storing R displacements
base_transfer_functions = tf(zeros(length(X_idx),1));
poles_base              = zeros(length(X_idx),5);
zeros_base              = zeros(length(X_idx),4);
nat_freqs_base          = zeros(length(X_idx),5);
J0_base                 = zeros(length(X_idx),1);
J1_base                 = zeros(length(X_idx),1);

%% Trouble shooting experiments - 4 different poses

R_base       = [245.455 654.545 654.545 241.667]*1/1000;
V_base       = [45 45 78.75 5.625]*2*pi/360;
R_shoulder   = [245.455 654.545 654.545 241.667]*1/1000;
V_shoulder   = [45 45 78.75 5.625]*2*pi/360;

base_TF     = tf(zeros(length(R_base),1));
shoulder_TF = base_TF;

cont_base   = 1;
cont_shouler= 1;

q_exp       = [0 -2.0771 2.3307 -0.2537 0 0; % pose 1
               0 -1.3327 0.9415 0.3912  0 0; % pose 2
               0 -1.784  0.8014 0.9826  0 0; % pose 3
               0 -1.5951 2.472 -0.8768  0 0];% pose 4

for k = 1:length(X_idx)
    filename = fullfile('D:\OneDrive - Umich\PhD\System ID 5 kg\FRFs\FRFs',dir_date_str,I{k});      
    load(filename);
    filename = I{k}; % remove the directory info
    
    H_B(:,k)      = reshape(H_Y1,[length(H_Y1),1]);
    H_B_full(:,k) = reshape(H_Y1_full,[length(H_Y1_full),1]);
    H_B_joint(:,k)= reshape(H_Y1_joint,[length(H_Y1_joint),1]);

%    find the joint positions by using the name of the file
    pos_idx = strfind(filename,'pos'); % returns a vector of the starting index
    end_idx = strfind(filename,date_str);
    sub_str = filename(pos_idx+4:end_idx-1);
    sub_str = replace(sub_str,'pt','.');

    spaces = strfind(sub_str,'_'); 
    q_ref  = zeros(1,6); % 6 joints on UR5e
    
    for j = 1:6
        if j == 1
            q_ref(j) = str2double(sub_str(1:spaces(j)-1));
        else
            q_ref(j) = str2double(sub_str(spaces(j-1)+1:spaces(j)-1));
        end
    end
    
    joint_positions_base(k,:) = q_ref;
    V_rad_base(k)             = V_rad;V_rad*180/pi; % loaded in from the file
    R_m_base(k)               = R_m;R_m;
    
    if V_rad*180/pi<40 || R_m >= 0.59
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Change here
        index_pole1      = find(freq_range_fit <= 30);
        num_order_base   = 2;
        den_order_base   = 3;
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

        w_fit            = 2*pi*freq_range_fit; % Frequncy vector to simulate the FRF of the model

        weight_pole1     = ones(length(index_pole1),1); % weight vector for fitting
        weight_pole1(1)  = 10^6; % weight of the first element must be big. 
      
%         First Pole
%         fitting the FRF
        H_base_fit    = horzcat(1, H_Y1); % Accounting for the extra 0.001 we added in the frequency vector
        [b_fit,a_fit] = invfreqs(H_base_fit(index_pole1),w_fit(index_pole1),num_order_base,den_order_base,weight_pole1(index_pole1),10^3);
        H_B_fit(:,k)  = freqs(b_fit,a_fit,w_fit); % Simulating the FRF of the model 
        sys           = tf(b_fit,a_fit);
        base_transfer_functions(k) = sys;
%       
%         get the poles and zeros of the fit in rad/s
        [p,z]               = pzmap(sys);
        poles_base(k,:)     = [p;0;0]; 
        zeros_base(k,:)     = [z;0;0];
        nat_freqs_base(k,:) = [abs(p);-1;-2]; % [rad/s] - 1 and -2 could be
%         any number. I  just want to avoind reorganizing the vectors.
%         -1 and -2 will be naturally excluded in the following steps.
     else
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Change here
        index_pole1        = find(freq_range_fit <= 40);
        num_order_base     = 4;
        den_order_base     = 5;
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        w_fit              = 2*pi*freq_range_fit; % Frequncy vector to simulate the FRF of the model

        weight_pole1       = ones(length(index_pole1),1); % weight vector for fitting
        weight_pole1(1)    = 10^6; % weight of the first element must be big. 
        
        %fitting the FRF
        H_base_fit    = horzcat(1, H_Y1); % Accounting for the extra 0.001 we added in the frequency vector
        [b_fit,a_fit] = invfreqs(H_base_fit(index_pole1),w_fit(index_pole1),num_order_base,den_order_base,weight_pole1(index_pole1),10^3);
        H_B_fit(:,k)  = freqs(b_fit,a_fit,w_fit); % Simulating the FRF of the model 
        sys           = tf(b_fit,a_fit);
        base_transfer_functions(k) = sys;
      
        %get the poles and zeros of the fit in rad/s
        [p,z]               = pzmap(sys);   
        poles_base(k,:)     = p; 
        zeros_base(k,:)     = z;
        nat_freqs_base(k,:) = abs(p); % [rad/s] - 1 and -2 could be
%         any number. I  just want to avoind reorganizing the vectors.
%         -1 and -2 will be naturally excluded in the following steps.
    end

% %     Second Pole
% %     fitting the FRF
%     [b_fit,a_fit] = invfreqs(H_base_fit(index_pole1),w_fit(index_pole1),num_order_base,den_order_base,weight(index_pole1),10^3);
%     H_B_fit(:,k)  = freqs(b_fit,a_fit,w_fit);
%     sys           = tf(b_fit,a_fit);
%     base_transfer_functions(k) = sys;
%   
%     % get the poles and zeros of the fit in rad/s
%     [p,z]               = pzmap(sys);
%     poles_base(k,:)     = p; 
%     zeros_base(k,:)     = z;
%     nat_freqs_base(k,:) = abs(p); % [rad/s]

%     
% %     for burro = 1:4   %  This code's section creates the V and R vectors 
% %                       %  of the deisered poses. This way, we can plot
% %                       %  selected data in the future.
% %        if q_ref == q_exp(burro,:)
% %         base_TF(cont_base) = base_transfer_functions(k);
% %         nat_freq           = abs(p)/(2*pi);
% %         zeta               = -real(p)/abs(p);
% %         R_base(cont_base)  = R_m;
% %         V_base(cont_base)  = V_rad;
% %         cont_base           = cont_base+1;
% %        
% %         q_exp  = [0 -2.0771 2.3307 -0.2537 0 0; % pose 1
% %                0 -1.3327 0.9415 0.3912  0 0; % pose 2
% %                0 -1.784  0.8014 0.9826  0 0; % pose 3
% %                0 -1.5951 2.472 -0.8768  0 0];% pose 4
% %          if q_ref == q_exp(1,:)
% %                 disp('Pose 1')
% %                  V_rad*360/(2*pi)
% %                 R_m*1000
% %          elseif q_ref == q_exp(2,:)
% %                 disp('Pose 2')
% %                  V_rad*360/(2*pi)
% %                 R_m*1000
% %          elseif q_ref == q_exp(3,:)
% %                 disp('Pose 3')
% %                  V_rad*360/(2*pi)
% %                 R_m*1000
% %          elseif q_ref == q_exp(4,:)
% %                 disp('Pose 4')
% %                 V_rad*360/(2*pi)
% %                 R_m*1000
% %          end
% %         figure(1)
% %         subplot(2,1,1)
% %         semilogx(freq_range,mag2db(abs(H_B(:,k))),'LineWidth',2);hold on
% %         semilogx(w_fit/(2*pi),mag2db(abs(H_B_fit(:,k))),'LineWidth',2);hold off
% %         ylabel('Mag [dB]')
% %         grid on
% %         xlim([2 60])
% %         title('Base FRFs')
% %       
% %         
% %         subplot(2,1,2)
% %         semilogx(freq_range,unwrap(angle(H_B(:,k)))/pi*180,'LineWidth',2);hold on
% %         semilogx(w_fit/(2*pi),unwrap(angle(H_B_fit(:,k)))/pi*180,'LineWidth',2);hold off
% %         xlabel('Frequency [Hz]')
% %         ylabel('Phase [deg]');
% %         grid on
% %         xlim([2 60]); 
% %         pause(0.01)
% % 
% %        end
% %    end
%     
     figure(2)
     subplot(2,3,1)
     semilogx(freq_range,mag2db(abs(H_B(:,k))),'LineWidth',2);hold on
     semilogx(w_fit/(2*pi),mag2db(abs(H_B_fit(:,k))),'--','LineWidth',2);hold off
     ylabel('Mag [dB]')
     grid on
     xlim([2 60])
     title(['Base FRFs R = ',num2str(R_m),' V = ',num2str(V_rad*180/pi),''])
     legend('sys ID','Fited','Location','southwest')
        
     subplot(2,3,4)
     semilogx(freq_range,unwrap(angle(H_B(:,k)))/pi*180,'LineWidth',2);hold on
     semilogx(w_fit/(2*pi),unwrap(angle(H_B_fit(:,k)))/pi*180,'--','LineWidth',2);hold off
     xlabel('Frequency [Hz]')
     ylabel('Phase [deg]');
     grid on
     xlim([2 60]); 


     subplot(2,3,2)
     semilogx(freq_range,mag2db(abs(H_B_full(:,k))),'LineWidth',2);hold on
     semilogx(freq_range,mag2db(abs(H_B_fit(2:end,k).*H_B_joint(1:end,k))),'--','LineWidth',2);hold off
     ylabel('Mag [dB]')
     grid on
     xlim([2 60])
     title(['Base Full FRFs R = ',num2str(R_m),' V = ',num2str(V_rad*180/pi),''])
     legend('sys ID','Fited','Location','southwest')
        
     subplot(2,3,5)
     semilogx(freq_range,unwrap(angle(H_B_full(:,k)))/pi*180,'LineWidth',2);hold on
     semilogx(freq_range,unwrap(angle(H_B_fit(2:end,k).*H_B_joint(1:end,k)))/pi*180,'--','LineWidth',2);hold off
     xlabel('Frequency [Hz]')
     ylabel('Phase [deg]');
     grid on
     xlim([2 60]); 
     

     subplot(2,3,3)
     semilogx(freq_range,mag2db(abs(H_B_joint(:,k))),'LineWidth',2);%hold on
     %semilogx(w_fit/(2*pi),mag2db(abs(H_B_fit(:,k))),'--','LineWidth',2);hold off
     ylabel('Mag [dB]')
     grid on
     xlim([2 60])
     title(['Base Joint FRFs R = ',num2str(R_m),' V = ',num2str(V_rad*180/pi),''])
     legend('sys ID','Fited','Location','southwest')
        
     subplot(2,3,6)
     semilogx(freq_range,unwrap(angle(H_B_joint(:,k)))/pi*180,'LineWidth',2);%hold on
     %semilogx(w_fit/(2*pi),unwrap(angle(H_B_fit(:,k)))/pi*180,'--','LineWidth',2);hold off
     xlabel('Frequency [Hz]')
     ylabel('Phase [deg]');
     grid on
     xlim([2 60]); 
%     pause(1)
%    
%    
%     %%%%%%%%%%%% This is used to generate a gif
% %     if k==1
% %         gif('Base_0_to_30Hz.gif','DelayTime',1/5)
% %     else
% %         gif
% %     end
%     %%%%%%%%%%%%
% 
    % store the base & shoulder inertia of the given configuration
    inertia_mat = ur5e_inertia_matrix_5kg(q_ref(1),q_ref(2),q_ref(3),q_ref(4),q_ref(5),q_ref(6));
    J0_base(k)  = inertia_mat(1,1);
    J1_base(k)  = inertia_mat(2,2);
% 
end
% 
% [B,r] = sort(R_m_base,'ascend'); %% Finding the index to put R in a ascending order
% [B,v] = sort(V_rad_base,'ascend'); %% Finding the index to put V in a ascending order

% % for k=1:length(R_m_base)
% % 
% %     index = r(k);
% % 
% %      figure(2)
% %      subplot(2,1,1)
% %      semilogx(freq_range,mag2db(abs(H_B(:,index))),'LineWidth',2);hold on
% %      semilogx(w_fit/(2*pi),mag2db(abs(H_B_fit(:,index))),'--','LineWidth',2);
% %      yline(-3,'b-.','-3 dB','LineWidth',2,'LabelHorizontalAlignment','left',...
% %          'LabelVerticalAlignment','bottom');hold off  % plots a line at 3 dB;hold off
% %      xline(45,'r-.','45 Hz','LineWidth',2,'LabelVerticalAlignment','bottom'...
% %          ,'LabelOrientation','horizontal','LabelHorizontalAlignment','left');hold off  % plots a line at 3 dB;hold off
% %      ylabel('Mag [dB]')  
% %      grid on
% %      xlim([2 60])
% %      title(['Base FRFs R = ',num2str(R_m_base(index)),' V = ',num2str(V_rad_base(index)*180/pi),'\color{darkGreen}{ FIXED R} \color{red}{--} \color{blue}{INCREASING V}'])
% %      legend('sys ID','Fited','Location','southwest')
% %         
% %      subplot(2,1,2)
% %      semilogx(freq_range,unwrap(angle(H_B(:,index)))/pi*180,'LineWidth',2);hold on
% %      semilogx(w_fit/(2*pi),unwrap(angle(H_B_fit(:,index)))/pi*180,'--','LineWidth',2);hold off
% %      xlabel('Frequency [Hz]')
% %      ylabel('Phase [deg]');
% %      grid on
% %      xlim([2 60]); 
% %      %pause(0.5)
% % %     if k==1
% % %         gif('Base_fixed_R.gif','DelayTime',1/3)
% % %     else
% % %         gif
% % %     end
% 
% 
% end
% 
% for k=1:length(V_rad_base)
% 
%     index = v(k);
% 
%     figure(2)
%     subplot(2,1,1)
%     semilogx(freq_range,mag2db(abs(H_B(:,index))),'LineWidth',2);hold on
%     semilogx(w_fit/(2*pi),mag2db(abs(H_B_fit(:,index))),'--','LineWidth',2);
%     yline(-3,'b-.','-3 dB','LineWidth',2,'LabelHorizontalAlignment','left',...
%          'LabelVerticalAlignment','bottom');hold off  % plots a line at 3 dB;hold off
%     xline(45,'r-.','45 Hz','LineWidth',2,'LabelVerticalAlignment','bottom'...
%          ,'LabelOrientation','horizontal','LabelHorizontalAlignment','left');hold off  % plots a line at 3 dB;hold off
%     ylabel('Mag [dB]')
%     grid on
%     xlim([2 60])
%     title(['Base FRFs R = ',num2str(R_m_base(index)),' V = ',num2str(V_rad_base(index)*180/pi),'\color{darkGreen}{ FIXED V} \color{blue}{--} \color{red}{DECREASING R}'])
%     legend('sys ID','Fited','Location','southwest')
%         
%     subplot(2,1,2)
%     semilogx(freq_range,unwrap(angle(H_B(:,index)))/pi*180,'LineWidth',2);hold on
%     semilogx(w_fit/(2*pi),unwrap(angle(H_B_fit(:,index)))/pi*180,'--','LineWidth',2);hold off
%     xlabel('Frequency [Hz]')
%     ylabel('Phase [deg]');
%     grid on
%     xlim([2 60]); 
% 
% %   pause(0.5)
% %     if k==1
% %         gif('Base_fixed_V.gif','DelayTime',1/3)
% %     else
% %         gif
% %     end
% 
% end

%save("Base_FRF.mat",'H_B')
%web('Base_fixed_V.gif') %This function is used to see the gif
%% Loading FRFs - shoulder
X = contains(N,'FRF_shoulder_'); % just 'shoulder' files
X_idx = find(X);
I = cell(length(X_idx),1);
for i = 1:length(X_idx)
    I{i} = N{X_idx(i)}; % shoulder filenames
end

H_S            = zeros(length(H_Y1),length(X_idx));       % shoulder should be the same frequency length as the base
H_S_fit        = zeros(length(freq_range_fit),length(X_idx));
index_shoulder = find(freq_range_fit <= 30);              % this is the range of interest to fit the shoulder FRFs

H_S_full     = zeros(length(H_Y1_full),length(X_idx)); 
H_S_joint    = zeros(length(H_Y1_joint),length(X_idx)); 

weight         = ones(length(index_shoulder),1); % weight vector for fitting
weight(1)      = 10^6; % weight of the first element must be big. 
w_fit          = 2*pi*freq_range_fit;

joint_positions_shoulder    = zeros(length(X_idx),6); % for storing joint positions
V_rad_shoulder              = zeros(length(X_idx),1); % for storing V angles
R_m_shoulder                = zeros(length(X_idx),1); % for storing R displacements
shoulder_transfer_functions = tf(zeros(length(X_idx),1));
poles_shoulder              = zeros(length(X_idx),den_order_shoulder);
zeros_shoulder              = zeros(length(X_idx),num_order_shoulder);
nat_freqs_shoulder          = zeros(length(X_idx),den_order_shoulder);
J1_shoulder                 = zeros(length(X_idx),1);
zero_wn                     = zeros(length(X_idx),num_order_shoulder);

for k = 1:length(X_idx)
    
    filename = fullfile('D:\OneDrive - Umich\PhD\System ID 5 kg\FRFs\FRFs',dir_date_str,I{k});
    load(filename);
    filename = I{k}; % remove the directory info
    H_S(:,k) = reshape(H_Y2,[length(H_Y2),1]); % H_Y2 is what the shoulder FRFs are saved as
    H_S_full(:,k) = reshape(H_Y2_full,[length(H_Y2_full),1]);
    H_S_joint(:,k)= reshape(H_Y2_joint,[length(H_Y2_joint),1]);


    % find the joint positions by parsing the string
    pos_idx = strfind(filename,'pos'); % returns a vector of the starting index
    end_idx = strfind(filename,date_str);
    sub_str = filename(pos_idx+4:end_idx-1);
    sub_str = replace(sub_str,'pt','.');

    spaces = strfind(sub_str,'_'); 
    q_ref  = zeros(1,6); % 6 joints on UR5e
    for j = 1:6
        if j == 1
            q_ref(j) = str2double(sub_str(1:spaces(j)-1));
        else
            q_ref(j) = str2double(sub_str(spaces(j-1)+1:spaces(j)-1));
        end
    end

    joint_positions_shoulder(k,:) = q_ref;
    V_rad_shoulder(k)             = V_rad; % loaded in from the file
    R_m_shoulder(k)               = R_m;

    % fitting the FRF
    H_shoulder_fit                 = horzcat(1, H_Y2);
    [b_fit,a_fit]                  = invfreqs(H_shoulder_fit(index_shoulder),w_fit(index_shoulder),num_order_shoulder,den_order_shoulder,weight(index_shoulder),10^3);
    H_S_fit(:,k)                   = freqs(b_fit,a_fit,w_fit);
    sys                            = tf(b_fit,a_fit);
    shoulder_transfer_functions(k) = sys;
    
    % get the poles and zeros of the fit in rad/s
    [p,z]                   = pzmap(sys);
    poles_shoulder(k,:)     = p;
    zeros_shoulder(k,:)     = z;
    nat_freqs_shoulder(k,:) = abs(p); % [rad/s]
    zero_wn(k,:)            = abs(z);

%      for burro = 1:4     
%        if q_ref == q_exp(burro,:)
%         shoulder_TF(cont_shouler) = shoulder_transfer_functions(k);
%         nat_freq                  = abs(p)/(2*pi);
%         zeta                      = -real(p)/abs(p);
%         R_shoulder(cont_shouler)  = R_m;
%         V_shoulder(cont_shouler)  = V_rad;
%         cont_shouler              = cont_shouler+1;
%         disp(cont_shouler)
%        end
%     end
    
        figure(3)
        subplot(2,1,1)
        semilogx(freq_range,mag2db(abs(H_S(:,k))),'LineWidth',2);hold on
        semilogx(w_fit/(2*pi),mag2db(abs(H_S_fit(:,k))),'--','LineWidth',2);hold off
        ylabel('Mag [dB]')
        grid on
        xlim([2 60])
        title(['Shoulder FRFs R = ',num2str(R_m),' V = ',num2str(V_rad*180/pi),''])
        legend('sys ID','Fited',"Location","southwest")
        
        subplot(2,1,2)
        semilogx(freq_range,unwrap(angle(H_S(:,k)))/pi*180,'LineWidth',2);hold on
        semilogx(w_fit/(2*pi),unwrap(angle(H_S_fit(:,k)))/pi*180,'--','LineWidth',2);hold off
        xlabel('Frequency [Hz]')
        ylabel('Phase [deg]');
        grid on
        xlim([2 60]); 
        pause(0.01)

    %%%%%%%%%%%% This is used to generate a gif
%     if k==1
%         gif('Shoulder.gif','DelayTime',1/5)
%     else
%         gif
%     end
    %%%%%%%%%%%%
    % store the base & shoulder inertia of the given configuration
    inertia_mat = ur5e_inertia_matrix_5kg(q_ref(1),q_ref(2),q_ref(3),q_ref(4),q_ref(5),q_ref(6));
    J1_shoulder(k) = inertia_mat(2,2);
end

%% Loading FRFs - Elbow
date_str     = '26-Jun-2023';
X            = contains(N,'FRF_elbow_'); % just 'shoulder' files
X_idx        = find(X);
I            = cell(length(X_idx),1);

for i = 1:length(X_idx)
    I{i} = N{X_idx(i)}; % shoulder filenames
end

H_E          = zeros(length(H_Y1),length(X_idx)); % shoulder should be the same frequency length as the base
H_E_fit      = zeros(length(freq_range_fit),length(X_idx));
index_elbow  = find(freq_range_fit <= 35); % this is the range of interest to fit the shoulder FRFs

H_E_full     = zeros(length(H_Y1_full),length(X_idx)); 
H_E_joint    = zeros(length(H_Y1_joint),length(X_idx)); 

weight       = ones(length(index_elbow),1); % weight vector for fitting
weight(1)    = 10^6; % weight of the first element must be big. 
w_fit        = 2*pi*freq_range_fit;

joint_positions_elbow    = zeros(length(X_idx),6); % for storing joint positions
V_rad_elbow              = zeros(length(X_idx),1); % for storing V angles
R_m_elbow                = zeros(length(X_idx),1); % for storing R displacements
elbow_transfer_functions = tf(zeros(length(X_idx),1));
poles_elbow              = zeros(length(X_idx),den_order_elbow);
zeros_elbow              = zeros(length(X_idx),num_order_elbow);
nat_freqs_elbow          = zeros(length(X_idx),den_order_elbow);
J2_elbow                 = zeros(length(X_idx),1);
zero_wn_elbow            = zeros(length(X_idx),num_order_elbow);

for k = 1:length(X_idx)
    
    filename = fullfile('D:\OneDrive - Umich\PhD\System ID 5 kg\FRFs\FRFs',dir_date_str,I{k});
    load(filename);
    filename = I{k}; % remove the directory info
    H_E(:,k) = reshape(H_Y3,[length(H_Y3),1]); % H_Y2 is what the shoulder FRFs are saved as
    H_E_full(:,k)  = reshape(H_Y3_full,[length(H_Y3_full),1]);
    H_E_joint(:,k) = reshape(H_Y3_joint,[length(H_Y3_joint),1]);

    % find the joint positions by parsing the string
    pos_idx = strfind(filename,'pos'); % returns a vector of the starting index
    end_idx = strfind(filename,date_str);
    sub_str = filename(pos_idx+4:end_idx-1);
    sub_str = replace(sub_str,'pt','.');

    spaces = strfind(sub_str,'_'); 
    q_ref  = zeros(1,6); % 6 joints on UR5e
    for j = 1:6
        if j == 1
            q_ref(j) = str2double(sub_str(1:spaces(j)-1));
        else
            q_ref(j) = str2double(sub_str(spaces(j-1)+1:spaces(j)-1));
        end
    end

    joint_positions_elbow(k,:) = q_ref;
    V_rad_elbow(k)             = V_rad; % loaded in from the file
    R_m_elbow(k)               = R_m;

    % fitting the FRF
    H_elbow_fit                 = horzcat(1, H_Y3);
    [b_fit,a_fit]               = invfreqs(H_elbow_fit(index_elbow),w_fit(index_elbow),num_order_elbow,den_order_elbow,weight(index_elbow),10^3);
    H_E_fit(:,k)                = freqs(b_fit,a_fit,w_fit);
    sys                         = tf(b_fit,a_fit);
    elbow_transfer_functions(k) = sys;
    
    % get the poles and zeros of the fit in rad/s
    [p,z]                = pzmap(sys);
    poles_elbow(k,:)     = p;
    zeros_elbow(k,:)     = z;
    nat_freqs_elbow(k,:) = abs(p); % [rad/s]
    zero_wn_elbow(k,:)   = abs(z);

%      for burro = 1:4     
%        if q_ref == q_exp(burro,:)
%         shoulder_TF(cont_shouler) = shoulder_transfer_functions(k);
%         nat_freq                  = abs(p)/(2*pi);
%         zeta                      = -real(p)/abs(p);
%         R_shoulder(cont_shouler)  = R_m;
%         V_shoulder(cont_shouler)  = V_rad;
%         cont_shouler              = cont_shouler+1;
%         disp(cont_shouler)
%        end
%     end
    
        figure(3)
        subplot(2,1,1)
        semilogx(freq_range,mag2db(abs(H_E(:,k))),'LineWidth',2);hold on
        semilogx(w_fit/(2*pi),mag2db(abs(H_E_fit(:,k))),'--','LineWidth',2);hold off
        ylabel('Mag [dB]')
        grid on
        xlim([2 60])
        title(['Elbow FRFs R = ',num2str(R_m),' V = ',num2str(V_rad*180/pi),''])
        legend('sys ID','Fited',"Location","southwest")
        
        subplot(2,1,2)
        semilogx(freq_range,unwrap(angle(H_E(:,k)))/pi*180,'LineWidth',2);hold on
        semilogx(w_fit/(2*pi),unwrap(angle(H_E_fit(:,k)))/pi*180,'--','LineWidth',2);hold off
        xlabel('Frequency [Hz]')
        ylabel('Phase [deg]');
        grid on
        xlim([2 60]); 
        pause(0.01)

    %%%%%%%%%%%% This is used to generate a gif
%     if k==1
%         gif('Shoulder.gif','DelayTime',1/5)
%     else
%         gif
%     end
    %%%%%%%%%%%%
    % store the base & shoulder inertia of the given configuration
    inertia_mat    = ur5e_inertia_matrix_5kg(q_ref(1),q_ref(2),q_ref(3),q_ref(4),q_ref(5),q_ref(6));
    J2_elbow(k) = inertia_mat(3,3);
end

% save("Inertia and joint angles","J0_base","J1_shoulder","J2_elbow","joint_positions_shoulder","joint_positions_base","joint_positions_elbow")

% [B,r] = sort(R_m_shoulder,'ascend'); %% Finding the index to put R in a ascending order
% [B,v] = sort(V_rad_shoulder,'ascend'); %% Finding the index to put V in a ascending order

% for k=1:length(R_m_shoulder)
% 
%     index = r(k);
% 
% figure(1)
%          subplot(2,1,1)
%          semilogx(freq_range,mag2db(abs(H_S(:,index))),'b','LineWidth',2);hold on
%          semilogx(w_fit/(2*pi),mag2db(abs(H_S_fit(:,index))),'r--','LineWidth',2);
%          semilogx(freq_range,mag2db(abs(H_B(:,index))),'LineWidth',2);hold on
%          semilogx(w_fit/(2*pi),mag2db(abs(H_B_fit(:,index))),'k:','LineWidth',2);
%          semilogx(freq_range,mag2db(abs(H_E(:,index))),'LineWidth',2);hold on
%          semilogx(w_fit/(2*pi),mag2db(abs(H_E_fit(:,index))),'-.','LineWidth',2);
%          
%          yline(-3,'b-.','-3 dB','LineWidth',2,'LabelHorizontalAlignment','left',...
%              'LabelVerticalAlignment','bottom');hold off  % plots a line at 3 dB;hold off
%          xline(45,'r-.','45 Hz','LineWidth',2,'LabelVerticalAlignment','bottom'...
%              ,'LabelOrientation','horizontal','LabelHorizontalAlignment','left');  % plots a line at 3 dB;hold off
%          
%          ylabel('Mag [dB]')
%          grid on
%          xlim([2 60])
%          title(['FRFs R = ',num2str(R_m_base(index)),' V = ',num2str(V_rad_base(index)*180/pi),'\color{darkGreen}{ FIXED V} \color{blue}{--} \color{red}{DECREASING R}'])
%          legend('H_S','H_S_{fit}','H_B','H_B_{fit}','H_E','H_E_{fit}','Location','southwest')
%             
%          subplot(2,1,2)
%          semilogx(freq_range,unwrap(angle(H_S(:,index)))/pi*180,'LineWidth',2);hold on
%          semilogx(w_fit/(2*pi),unwrap(angle(H_S_fit(:,index)))/pi*180,'--','LineWidth',2);
%          semilogx(freq_range,unwrap(angle(H_B(:,index)))/pi*180,'LineWidth',2);
%          semilogx(w_fit/(2*pi),unwrap(angle(H_B_fit(:,index)))/pi*180,'k:','LineWidth',2);
%          semilogx(freq_range,unwrap(angle(H_E(:,index)))/pi*180,'LineWidth',2);
%          semilogx(w_fit/(2*pi),unwrap(angle(H_E_fit(:,index)))/pi*180,'-.','LineWidth',2);hold off
%          xlabel('Frequency [Hz]')
%          ylabel('Phase [deg]');
%          grid on
%          xlim([2 60]);
% % %     if k==1
% % %         gif('FRFs_fixed_R.gif','DelayTime',1/3)
% % %     else
% % %         gif
% % %     end
% % 
% end

cont = 1;
%%
% for k=1:length(V_rad_shoulder) % Plot fixing V and changing R
% 
%     index = v(k);
%     
%    % if V_rad_base(index)*180/pi>60
%          figure(2)
%          subplot(2,1,1)
%           semilogx(freq_range,mag2db(abs(H_S(:,index))),'b','LineWidth',2);hold on
%           semilogx(w_fit/(2*pi),mag2db(abs(H_S_fit(:,index))),'r--','LineWidth',2);
%           semilogx(freq_range,mag2db(abs(H_B(:,index))),'LineWidth',2);hold on
%           semilogx(w_fit/(2*pi),mag2db(abs(H_B_fit(:,index))),'k:','LineWidth',2);
%          semilogx(freq_range,mag2db(abs(H_E(:,index))),'LineWidth',2);hold on
%          semilogx(w_fit/(2*pi),mag2db(abs(H_E_fit(:,index))),'-.','LineWidth',2);
%          
%          yline(-3,'b-.','-3 dB','LineWidth',2,'LabelHorizontalAlignment','left',...
%              'LabelVerticalAlignment','bottom');hold off  % plots a line at 3 dB;hold off
%          xline(45,'r-.','45 Hz','LineWidth',2,'LabelVerticalAlignment','bottom'...
%              ,'LabelOrientation','horizontal','LabelHorizontalAlignment','left');  % plots a line at 3 dB;hold off
%          
%          ylabel('Mag [dB]')
%          grid on
%          xlim([2 60])
%          title(['FRFs R = ',num2str(R_m_base(index)),' V = ',num2str(V_rad_base(index)*180/pi),'\color{darkGreen}{ FIXED V} \color{blue}{--} \color{red}{DECREASING R}'])
%          legend('H_S','H_S_{fit}','H_B','H_B_{fit}','H_E','H_E_{fit}','Location','southwest')
%          %legend('H_E','H_E_{fit}','Location','southwest')
%             
%          subplot(2,1,2)
% %          semilogx(freq_range,unwrap(angle(H_S(:,index)))/pi*180,'LineWidth',2);hold on
% %          semilogx(w_fit/(2*pi),unwrap(angle(H_S_fit(:,index)))/pi*180,'--','LineWidth',2);
% %          semilogx(freq_range,unwrap(angle(H_B(:,index)))/pi*180,'LineWidth',2);
% %          semilogx(w_fit/(2*pi),unwrap(angle(H_B_fit(:,index)))/pi*180,'k:','LineWidth',2);
%          semilogx(freq_range,unwrap(angle(H_E(:,index)))/pi*180,'LineWidth',2);hold on
%          semilogx(w_fit/(2*pi),unwrap(angle(H_E_fit(:,index)))/pi*180,'-.','LineWidth',2);hold off
%          xlabel('Frequency [Hz]')
%          ylabel('Phase [deg]');
%          grid on
%          xlim([2 60]);
%          pause(0.1)
% %          if cont ==1
% %              gif('FRFs_fixed_V_60_2_90.gif','DelayTime',1/2)
% %              cont = 2;
% %          else
% %              gif
% %          end
% %     end
%       
% %     if k==1
% %         gif('FRFs_fixed_V.gif','DelayTime',1/3)
% %     else
% %         gif
% %     end
% 
%    
% end
% web('FRFs_fixed_V_70_2_90.gif') %This function is used to see the gif

%% Plot H,H_full, H_joint

% for k=1:length(V_rad_shoulder) % Plot fixing V and changing R
% 
%     index = v(k);
%     
%    % if V_rad_base(index)*180/pi>60
%          figure(2)
%          subplot(2,4,1)
%          semilogx(freq_range,mag2db(abs(H_B(:,index))),'LineWidth',2);
%          
%          yline(-3,'b-.','-3 dB','LineWidth',2,'LabelHorizontalAlignment','left',...
%              'LabelVerticalAlignment','bottom');hold off  % plots a line at 3 dB;hold off
% %          xline(45,'r-.','45 Hz','LineWidth',2,'LabelVerticalAlignment','bottom'...
% %              ,'LabelOrientation','horizontal','LabelHorizontalAlignment','left');  % plots a line at 3 dB;hold off
%          
%          ylabel('Mag [dB]')
%          grid on
%          xlim([2 60])
%          title(['H_B - FRFs R = ',num2str(R_m_base(index)),' V = ',num2str(V_rad_base(index)*180/pi),'\color{darkGreen}{ FIXED V} \color{blue}{--} \color{red}{DECREASING R}'])
%          legend('H_B','Location','southwest')
%            
%          subplot(2,4,5)
%          semilogx(freq_range,unwrap(angle(H_B(:,index)))/pi*180,'LineWidth',2);
%          xlabel('Frequency [Hz]')
%          ylabel('Phase [deg]');
%          grid on
%          xlim([2 60]);
%         
%          
%          subplot(2,4,3)
%          semilogx(freq_range,mag2db(abs(H_B_full(:,index))),'r:','LineWidth',2);hold on
%          
%          yline(-3,'b-.','-3 dB','LineWidth',2,'LabelHorizontalAlignment','left',...
%              'LabelVerticalAlignment','bottom');hold off  % plots a line at 3 dB;hold off
% %          xline(45,'r-.','45 Hz','LineWidth',2,'LabelVerticalAlignment','bottom'...
% %              ,'LabelOrientation','horizontal','LabelHorizontalAlignment','left');  % plots a line at 3 dB;hold off
%          
%          ylabel('Mag [dB]')
%          grid on
%          xlim([2 60])
%          title(['H_B_{full} - FRFs R = ',num2str(R_m_base(index)),' V = ',num2str(V_rad_base(index)*180/pi),''])
%          legend('H_B_{full}','Location','southwest')
%            
%          subplot(2,4,7)
%          semilogx(freq_range,unwrap(angle(H_B_full(:,index)))/pi*180,'r:','LineWidth',2);
%          xlabel('Frequency [Hz]')
%          ylabel('Phase [deg]');
%          grid on
%          xlim([2 60]);
% 
%          subplot(2,4,2)
%          semilogx(freq_range,mag2db(abs(H_B_joint(:,index))),'k-.','LineWidth',2);hold on
%          
%          yline(-3,'b-.','-3 dB','LineWidth',2,'LabelHorizontalAlignment','left',...
%              'LabelVerticalAlignment','bottom');hold off  % plots a line at 3 dB;hold off
% %          xline(45,'r-.','45 Hz','LineWidth',2,'LabelVerticalAlignment','bottom'...
% %              ,'LabelOrientation','horizontal','LabelHorizontalAlignment','left');  % plots a line at 3 dB;hold off
%          
%          ylabel('Mag [dB]')
%          grid on
%          xlim([2 60])
%          title(['H_B_{joint} - FRFs R = ',num2str(R_m_base(index)),' V = ',num2str(V_rad_base(index)*180/pi),''])
%          legend('H_B_{joint}','Location','southwest')
%            
%          subplot(2,4,6)
%          semilogx(freq_range,unwrap(angle(H_B_joint(:,index)))/pi*180,'k-.','LineWidth',2);
%          xlabel('Frequency [Hz]')
%          ylabel('Phase [deg]');
%          grid on
%          xlim([2 60]);
% 
% 
%          subplot(2,4,4)
%          semilogx(freq_range,mag2db(abs(H_B(:,index).*H_B_joint(:,index))),'g','LineWidth',2);hold on
%          
%          yline(-3,'b-.','-3 dB','LineWidth',2,'LabelHorizontalAlignment','left',...
%              'LabelVerticalAlignment','bottom');hold off  % plots a line at 3 dB;hold off
% %          xline(45,'r-.','45 Hz','LineWidth',2,'LabelVerticalAlignment','bottom'...
% %              ,'LabelOrientation','horizontal','LabelHorizontalAlignment','left');  % plots a line at 3 dB;hold off
%          
%          ylabel('Mag [dB]')
%          grid on
%          xlim([2 60])
%          title(['H_B*H_B_{joint} - FRFs R = ',num2str(R_m_base(index)),' V = ',num2str(V_rad_base(index)*180/pi),''])
%          legend('H_B*H_B_{joint}','Location','southwest')
%            
%          subplot(2,4,8)
%          semilogx(freq_range,unwrap(angle(H_B(:,index).*H_B_joint(:,index)))/pi*180,'g','LineWidth',2);
%          xlabel('Frequency [Hz]')
%          ylabel('Phase [deg]');
%          grid on
%          xlim([2 60]);
%       
% %     if k==1
% %         gif('Base_FRFs.gif','DelayTime',1/3)
% %     else
% %         gif
% %     end
%    
% end


% for k=1:length(V_rad_shoulder) % Plot fixing V and changing R
% 
%     index = v(k);
%     
%    % if V_rad_base(index)*180/pi>60
%          figure(2)
%          subplot(2,4,1)
%          semilogx(freq_range,mag2db(abs(H_S(:,index))),'LineWidth',2);
%          
%          yline(-3,'b-.','-3 dB','LineWidth',2,'LabelHorizontalAlignment','left',...
%              'LabelVerticalAlignment','bottom');hold off  % plots a line at 3 dB;hold off
% %          xline(45,'r-.','45 Hz','LineWidth',2,'LabelVerticalAlignment','bottom'...
% %              ,'LabelOrientation','horizontal','LabelHorizontalAlignment','left');  % plots a line at 3 dB;hold off
%          
%          ylabel('Mag [dB]')
%          grid on
%          xlim([2 60])
%          title(['H_S - FRFs R = ',num2str(R_m_base(index)),' V = ',num2str(V_rad_base(index)*180/pi),'\color{darkGreen}{ FIXED V} \color{blue}{--} \color{red}{DECREASING R}'])
%          legend('H_S','Location','southwest')
%            
%          subplot(2,4,5)
%          semilogx(freq_range,unwrap(angle(H_S(:,index)))/pi*180,'LineWidth',2);
%          xlabel('Frequency [Hz]')
%          ylabel('Phase [deg]');
%          grid on
%          xlim([2 60]);
%         
%          
%          subplot(2,4,3)
%          semilogx(freq_range,mag2db(abs(H_S_full(:,index))),'r:','LineWidth',2);hold on
%          
%          yline(-3,'b-.','-3 dB','LineWidth',2,'LabelHorizontalAlignment','left',...
%              'LabelVerticalAlignment','bottom');hold off  % plots a line at 3 dB;hold off
% %          xline(45,'r-.','45 Hz','LineWidth',2,'LabelVerticalAlignment','bottom'...
% %              ,'LabelOrientation','horizontal','LabelHorizontalAlignment','left');  % plots a line at 3 dB;hold off
%          
%          ylabel('Mag [dB]')
%          grid on
%          xlim([2 60])
%          title(['H_S_{full} - FRFs R = ',num2str(R_m_base(index)),' V = ',num2str(V_rad_base(index)*180/pi),''])
%          legend('H_S_{full}','Location','southwest')
%            
%          subplot(2,4,7)
%          semilogx(freq_range,unwrap(angle(H_S_full(:,index)))/pi*180,'r:','LineWidth',2);
%          xlabel('Frequency [Hz]')
%          ylabel('Phase [deg]');
%          grid on
%          xlim([2 60]);
% 
%          subplot(2,4,2)
%          semilogx(freq_range,mag2db(abs(H_S_joint(:,index))),'k-.','LineWidth',2);hold on
%          
%          yline(-3,'b-.','-3 dB','LineWidth',2,'LabelHorizontalAlignment','left',...
%              'LabelVerticalAlignment','bottom');hold off  % plots a line at 3 dB;hold off
% %          xline(45,'r-.','45 Hz','LineWidth',2,'LabelVerticalAlignment','bottom'...
% %              ,'LabelOrientation','horizontal','LabelHorizontalAlignment','left');  % plots a line at 3 dB;hold off
%          
%          ylabel('Mag [dB]')
%          grid on
%          xlim([2 60])
%          title(['H_S_{joint} - FRFs R = ',num2str(R_m_base(index)),' V = ',num2str(V_rad_base(index)*180/pi),''])
%          legend('H_S_{joint}','Location','southwest')
%            
%          subplot(2,4,6)
%          semilogx(freq_range,unwrap(angle(H_S_joint(:,index)))/pi*180,'k-.','LineWidth',2);
%          xlabel('Frequency [Hz]')
%          ylabel('Phase [deg]');
%          grid on
%          xlim([2 60]);
% 
% 
%          subplot(2,4,4)
%          semilogx(freq_range,mag2db(abs(H_S(:,index).*H_S_joint(:,index))),'g','LineWidth',2);hold on
%          
%          yline(-3,'b-.','-3 dB','LineWidth',2,'LabelHorizontalAlignment','left',...
%              'LabelVerticalAlignment','bottom');hold off  % plots a line at 3 dB;hold off
% %          xline(45,'r-.','45 Hz','LineWidth',2,'LabelVerticalAlignment','bottom'...
% %              ,'LabelOrientation','horizontal','LabelHorizontalAlignment','left');  % plots a line at 3 dB;hold off
%          
%          ylabel('Mag [dB]')
%          grid on
%          xlim([2 60])
%          title(['H_S*H_S_{joint} - FRFs R = ',num2str(R_m_base(index)),' V = ',num2str(V_rad_base(index)*180/pi),''])
%          legend('H_S*H_S_{joint}','Location','southwest')
%            
%          subplot(2,4,8)
%          semilogx(freq_range,unwrap(angle(H_S(:,index).*H_S_joint(:,index)))/pi*180,'g','LineWidth',2);
%          xlabel('Frequency [Hz]')
%          ylabel('Phase [deg]');
%          grid on
%          xlim([2 60]);
%       
% %     if k==1
% %         gif('Shoulder_FRFs.gif','DelayTime',1/3)
% %     else
% %         gif
% %     end
% %    
% end
% 
% for k=1:length(V_rad_shoulder) % Plot fixing V and changing R
% 
%     index = v(k);
%     
%    % if V_rad_base(index)*180/pi>60
%          figure(2)
%          subplot(2,4,1)
%          semilogx(freq_range,mag2db(abs(H_E(:,index))),'LineWidth',2);
%          
%          yline(-3,'b-.','-3 dB','LineWidth',2,'LabelHorizontalAlignment','left',...
%              'LabelVerticalAlignment','bottom');hold off  % plots a line at 3 dB;hold off
% %          xline(45,'r-.','45 Hz','LineWidth',2,'LabelVerticalAlignment','bottom'...
% %              ,'LabelOrientation','horizontal','LabelHorizontalAlignment','left');  % plots a line at 3 dB;hold off
%          
%          ylabel('Mag [dB]')
%          grid on
%          xlim([2 60])
%          title(['H_E - FRFs R = ',num2str(R_m_base(index)),' V = ',num2str(V_rad_base(index)*180/pi),''])
%          legend('H_E','Location','southwest')
%            
%          subplot(2,4,5)
%          semilogx(freq_range,unwrap(angle(H_E(:,index)))/pi*180,'LineWidth',2);
%          xlabel('Frequency [Hz]')
%          ylabel('Phase [deg]');
%          grid on
%          xlim([2 60]);
%         
%          
%          subplot(2,4,3)
%          semilogx(freq_range,mag2db(abs(H_E_full(:,index))),'r:','LineWidth',2);hold on
%          
%          yline(-3,'b-.','-3 dB','LineWidth',2,'LabelHorizontalAlignment','left',...
%              'LabelVerticalAlignment','bottom');hold off  % plots a line at 3 dB;hold off
% %          xline(45,'r-.','45 Hz','LineWidth',2,'LabelVerticalAlignment','bottom'...
% %              ,'LabelOrientation','horizontal','LabelHorizontalAlignment','left');  % plots a line at 3 dB;hold off
%          
%          ylabel('Mag [dB]')
%          grid on
%          xlim([2 60])
%          title(['H_E_{full} - FRFs R = ',num2str(R_m_base(index)),' V = ',num2str(V_rad_base(index)*180/pi),''])
%          legend('H_E_{full}','Location','southwest')
%            
%          subplot(2,4,7)
%          semilogx(freq_range,unwrap(angle(H_E_full(:,index)))/pi*180,'r:','LineWidth',2);
%          xlabel('Frequency [Hz]')
%          ylabel('Phase [deg]');
%          grid on
%          xlim([2 60]);
% 
%          subplot(2,4,2)
%          semilogx(freq_range,mag2db(abs(H_E_joint(:,index))),'k-.','LineWidth',2);hold on
%          
%          yline(-3,'b-.','-3 dB','LineWidth',2,'LabelHorizontalAlignment','left',...
%              'LabelVerticalAlignment','bottom');hold off  % plots a line at 3 dB;hold off
% %          xline(45,'r-.','45 Hz','LineWidth',2,'LabelVerticalAlignment','bottom'...
% %              ,'LabelOrientation','horizontal','LabelHorizontalAlignment','left');  % plots a line at 3 dB;hold off
%          
%          ylabel('Mag [dB]')
%          grid on
%          xlim([2 60])
%          title(['H_E_{joint} - FRFs R = ',num2str(R_m_base(index)),' V = ',num2str(V_rad_base(index)*180/pi),'\color{darkGreen}{ FIXED V} \color{blue}{--} \color{red}{DECREASING R}'])
%          legend('H_E_{joint}','Location','southwest')
%            
%          subplot(2,4,6)
%          semilogx(freq_range,unwrap(angle(H_E_joint(:,index)))/pi*180,'k-.','LineWidth',2);
%          xlabel('Frequency [Hz]')
%          ylabel('Phase [deg]');
%          grid on
%          xlim([2 60]);
% 
%          subplot(2,4,4)
%          semilogx(freq_range,mag2db(abs(H_E(:,index).*H_E_joint(:,index))),'g','LineWidth',2);hold on
%          
%          yline(-3,'b-.','-3 dB','LineWidth',2,'LabelHorizontalAlignment','left',...
%              'LabelVerticalAlignment','bottom');hold off  % plots a line at 3 dB;hold off
% %          xline(45,'r-.','45 Hz','LineWidth',2,'LabelVerticalAlignment','bottom'...
% %              ,'LabelOrientation','horizontal','LabelHorizontalAlignment','left');  % plots a line at 3 dB;hold off
%          
%          ylabel('Mag [dB]')
%          grid on
%          xlim([2 60])
%          title(['H_E*H_E_{joint} - FRFs R = ',num2str(R_m_base(index)),' V = ',num2str(V_rad_base(index)*180/pi),''])
%          legend('H_E*H_E_{joint}','Location','southwest')
%            
%          subplot(2,4,8)
%          semilogx(freq_range,unwrap(angle(H_E(:,index).*H_E_joint(:,index)))/pi*180,'g','LineWidth',2);
%          xlabel('Frequency [Hz]')
%          ylabel('Phase [deg]');
%          grid on
%          xlim([2 60]);
%       
%       
% %     if k==1
% %         gif('Elbow_FRFs.gif','DelayTime',1/3)
% %     else
% %         gif
% %     end
% %    
% end

%save('Full and joint FRFs','H_E_joint','H_E_full','H_S_joint','H_S_full','H_B_full','H_B_joint')
%save('Four_pouses_FRF','R_shoulder','V_shoulder','R_base','V_base','shoulder_TF','base_TF')
%% Plot

% for i = 1:size(H_B,2)
%     if i ~= 1
%         delete(c2)
%         delete(h2)
%         delete(c3)
%     end
%     figure(11)
%     subplot(2,1,1)
%     c1=semilogx(freq_range,mag2db(abs(H_B(:,i))),'LineWidth',2);
%     hold on
%     c2=semilogx(freq_range_fit,mag2db(abs(H_B_fit(:,i))),'k--','LineWidth',2);
%     c3=text(3, -40, ['q_1: ', num2str(round(joint_positions_base(i,1),2)), ', q_2: ', num2str(round(joint_positions_base(i,2),2)),...
%         ', q_3: ', num2str(round(joint_positions_base(i,3),2)), ', q_4: ', num2str(round(joint_positions_base(i,4),2)),... 
%         ', q_5: ', num2str(round(joint_positions_base(i,5),2)), ', q_6: ', num2str(round(joint_positions_base(i,6),2))]);
%     ylabel('Mag [dB]')
%     grid on
%     xlim([2 60])
%     title('Base Joint FRFs (Measured & Fits)')
% %     hold off
% 
%     subplot(2,1,2)
%     h1=semilogx(freq_range,unwrap(angle(H_B(:,i)))/pi*180,'LineWidth',2);
%     hold on
%     h2=semilogx(freq_range_fit,unwrap(angle(H_B_fit(:,i)))/pi*180,'k--','LineWidth',2);
%     xlabel('Frequency [Hz]')
%     ylabel('Phase [deg]')
%     grid on
%     xlim([2 60])
% %     hold off
%     F(i) = getframe(gcf);
%     drawnow
%     pause(0.1);
% 
% end
% 
% % create the video writer with 1 fps
% writerObj = VideoWriter('base_frfs_5kg_video.avi');
% writerObj.FrameRate = 2;
% 
% % open the video writer
% open(writerObj);
% % write the frames to the video
% for i=1:length(F)
%     % convert the image to a frame
%     frame = F(i) ;    
%     writeVideo(writerObj, frame);
% end
% % close the writer object
% close(writerObj);
% 
% clear F 
% 
% for i = 1:size(H_S,2)
%     if i ~= 1
%         delete(c2)
%         delete(h2)
%         delete(c3)
%     end
%     figure(12)
%     subplot(2,1,1)
%     c1=semilogx(freq_range,mag2db(abs(H_S(:,i))),'LineWidth',2);
%     hold on
%     c2=semilogx(freq_range_fit,mag2db(abs(H_S_fit(:,i))),'k--','LineWidth',2);
%     c3=text(3, -24, ['q_1: ', num2str(round(joint_positions_shoulder(i,1),2)), ', q_2: ', num2str(round(joint_positions_shoulder(i,2),2)),...
%         ', q_3: ', num2str(round(joint_positions_shoulder(i,3),2)), ', q_4: ', num2str(round(joint_positions_shoulder(i,4),2)),... 
%         ', q_5: ', num2str(round(joint_positions_shoulder(i,5),2)), ', q_6: ', num2str(round(joint_positions_shoulder(i,6),2))]);
%     ylabel('Mag [dB]')
%     grid on
%     xlim([2 60])
%     title('Shoulder Joint FRFs (Measured & Fits)')
% %     hold off
% 
%     subplot(2,1,2)
%     h1=semilogx(freq_range,unwrap(angle(H_S(:,i)))/pi*180,'LineWidth',2);
%     hold on
%     h2=semilogx(freq_range_fit,unwrap(angle(H_S_fit(:,i)))/pi*180,'k--','LineWidth',2);
%     xlabel('Frequency [Hz]')
%     ylabel('Phase [deg]')
%     grid on
%     xlim([2 60])
% %     hold off
%     F(i) = getframe(gcf);
%     drawnow
%     pause(0.01);
% end
% 
% % create the video writer with 1 fps
% writerObj = VideoWriter('shoulder_frfs_5kg_video.avi');
% writerObj.FrameRate = 2;
% 
% % open the video writer
% open(writerObj);
% % write the frames to the video
% for i=1:length(F)
%     % convert the image to a frame
%     frame = F(i) ;    
%     writeVideo(writerObj, frame);
% end
% % close the writer object
% close(writerObj);
% 
% clear F writerObj 

%%
% figure(1)
% subplot(2,1,1)
% semilogx(freq_range,mag2db(abs(H_B)),'LineWidth',2)
% ylabel('Mag [dB]')
% grid on
% xlim([2 60])
% title('Base Joint FRFs (Measured)')
% % legend('[0, 0, 0, 0, 0, 0]', '[0, -\pi/2, 0, -\pi/2, 0, 0]', '[\pi/2, -\pi/2, 0, -\pi/2, 0, 0]','[\pi/2, -\pi/2, 5\pi/6, -\pi/2, 0, 0]', 'Location', 'southwest')
% 
% subplot(2,1,2)
% semilogx(freq_range,unwrap(angle(H_B))/pi*180,'LineWidth',2)
% xlabel('Frequency [Hz]')
% ylabel('Phase [deg]')
% grid on
% xlim([2 60])
% 
% figure(100)
% subplot(2,1,1)
% semilogx(freq_range_fit,mag2db(abs(H_B_fit)),'LineWidth',2)
% ylabel('Mag [dB]')
% grid on
% xlim([2 60])
% title('Base Joint FRFs (Fitted)')
% % legend('[0, 0, 0, 0, 0, 0]', '[0, -\pi/2, 0, -\pi/2, 0, 0]', '[\pi/2, -\pi/2, 0, -\pi/2, 0, 0]','[\pi/2, -\pi/2, 5\pi/6, -\pi/2, 0, 0]', 'Location', 'southwest')

% subplot(2,1,2)
% semilogx(freq_range_fit,unwrap(angle(H_B_fit))/pi*180,'LineWidth',2)
% xlabel('Frequency [Hz]')
% ylabel('Phase [deg]')
% grid on
% xlim([2 60])
% 
% 
% figure(2)
% subplot(2,1,1)
% semilogx(freq_range,mag2db(abs(H_S)),'LineWidth',2)
% ylabel('Mag [dB]')
% grid on
% xlim([2 60])
% title('Shoulder Joint FRFs (Measured)')
% 
% subplot(2,1,2)
% semilogx(freq_range,unwrap(angle(H_S))/pi*180,'LineWidth',2)
% xlabel('Frequency [Hz]')
% ylabel('Phase [deg]')
% grid on
% xlim([2 60])
% 
% figure(200)
% subplot(2,1,1)
% semilogx(freq_range_fit,mag2db(abs(H_S_fit)),'LineWidth',2)
% ylabel('Mag [dB]')
% grid on
% xlim([2 60])
% title('Shoulder Joint FRFs (Fitted)')
% 
% subplot(2,1,2)
% semilogx(freq_range_fit,unwrap(angle(H_S_fit))/pi*180,'LineWidth',2)
% xlabel('Frequency [Hz]')
% ylabel('Phase [deg]')
% grid on
% xlim([2 60])

%% Selecting and plotting natural frequencies
% Base joint
nat_freqs_base_high     = zeros(length(nat_freqs_base),1);
index_nat_freq_base     = zeros(length(nat_freqs_base),1);

nat_freqs_base_max      = zeros(length(nat_freqs_base),1); % Second poles of the system 
index_nat_freq_base_max = zeros(length(nat_freqs_base),1); 

for i = 1:length(nat_freqs_base)
   
    nat_freqs = nat_freqs_base(i,:);
    [~,idx]   = unique(nat_freqs_base(i,:),'stable');
    unique_nat_freqs_base = [];
    indices = [];


    for k = 1:length(idx)
        if idx(k)+1 <= length(nat_freqs)
            if nat_freqs(idx(k)+1) == nat_freqs(idx(k))
                unique_nat_freqs_base = horzcat(unique_nat_freqs_base,nat_freqs(idx(k)));
                indices = horzcat(indices,idx(k));
            end
        end
    end

    [nat_freqs_base_high(i),ind] = min(unique_nat_freqs_base);
    index_nat_freq_base(i)       = indices(ind);
    
    [nat_freqs_base_max(i),ind_max]  = max(unique_nat_freqs_base); % [M,I] = min(A) returns the index of the minimal value in A as well as the value.  
    index_nat_freq_base_max(i)       = indices(ind_max);
    
    
    if nat_freqs_base_high(i)/(2*pi) > 90 && min(unique_nat_freqs_base) ~= nat_freqs_base_high(i)
        [nat_freqs_base_high(i),ind] = min(unique_nat_freqs_base);
        index_nat_freq_base(i) = indices(ind);
    end
end

% compute damping ratios
zeta_base        = zeros(length(nat_freqs_base),1);
zeta_base_max    = zeta_base;
for i = 1:length(nat_freqs_base)
   
    pole         = poles_base(i,index_nat_freq_base(i));
    wn           = abs(pole);
    sigma        = -real(pole);
    zeta_base(i) = sigma/wn;

    pole_max_base        = poles_base(i,index_nat_freq_base_max(i));
    wn_max_base          = abs(pole_max_base);
    sigma_max_base       = -real(pole_max_base);
    zeta_base_max(i)     = sigma_max_base/wn_max_base;
end

% %% Excluding the second poles that are equal to the first poles
second_pole_index = find(nat_freqs_base_max~=nat_freqs_base_high);

figure(4)
plot3(V_rad_base*(180/pi),R_m_base*1000,nat_freqs_base_high/(2*pi), '*','LineWidth',2)
xlabel('Angle from horizontal, V [deg]')
ylabel('Radius from base, R [mm]')
zlabel('Natural frequency [Hz]')
grid on
title('base Joint Ressonance - First Pole')
%zlim([14 20])

figure(5)
plot3(V_rad_base*(180/pi),R_m_base*1000,zeta_base, '*','LineWidth',2)
xlabel('Angle from horizontal, V [deg]')
ylabel('Radius from base, R [mm]')
zlabel('damping ration')
grid on
title('base Joint damping ration - First pole')


figure(6)
plot3(V_rad_base(second_pole_index)*(180/pi),R_m_base(second_pole_index)*1000,nat_freqs_base_max(second_pole_index)/(2*pi), '*','LineWidth',2)
xlabel('Angle from horizontal, V [deg]')
ylabel('Radius from base, R [mm]')
zlabel('Natural frequency [Hz]')
grid on
title('base Joint Ressonance - Second Pole')
zlim([32 45])

figure(7)
plot3(V_rad_base(second_pole_index)*(180/pi),R_m_base(second_pole_index)*1000,zeta_base_max(second_pole_index), '*','LineWidth',2)
xlabel('Angle from horizontal, V [deg]')
ylabel('Radius from base, R [mm]')
zlabel('damping ration')
grid on
title('base Joint damping ration - Second pole of base FRF')

figure(8)
plot3(V_rad_base*(180/pi),R_m_base*1000,zeta_base, '*','LineWidth',2)
xlabel('Angle from horizontal, V [deg]')
ylabel('Radius from base, R [mm]')
zlabel('damping ration')
grid on
title('Base Joint damping ration - First pole')


%% Edited 05/30/2023
%%%%%%%%%%%%% Shoulder joint
nat_freqs_shoulder_high = zeros(length(nat_freqs_shoulder),1);     % Firts poles of the system 
index_nat_freq_shoulder = zeros(length(nat_freqs_shoulder),1);

nat_freqs_shoulder_max      = zeros(length(nat_freqs_shoulder),1); % Second poles of the system 
index_nat_freq_shoulder_max = zeros(length(nat_freqs_shoulder),1);

non_complex_conjugate_pole       = zeros(length(nat_freqs_shoulder),1);
index_non_complex_conjugate_pole = zeros(length(nat_freqs_shoulder),1);

% I noticed we normally have 2 non_complex_conjugate zeros
z_non_complex_conjugate          = zeros(length(nat_freqs_elbow),2);

z_complex_conjugate_1            = [];
z_complex_conjugate_2            = [];
z_non_complex_conjugate          = [];
cont_2_complex_conjugate         = [];

for i = 1:length(nat_freqs_shoulder)
    nat_freqs = nat_freqs_shoulder(i,:);
    [~,idx]   = unique(nat_freqs_shoulder(i,:),'stable'); % this function identify unique values in the arrays.
    %[~,idz,~] = unique(zero_wn(i,:),'first','legacy');
    indices   = [];

    unique_nat_freqs_shoulder  = [];
    unique_zero_wn_shoulder    = []; % horzcat concatenates nat_freqs(idx(k)) into unique_nat_freqs_shoulder
    indices_z                  = [];
    

   for k = 1:length(idx) % idx are the indices of the poles that appers once.
   
   % Note that if the index is grather than the length of
   % of the vector of poles, than it can not be a natural frequency because
   % natural frequencies appears in pairs. EX: if you have a 5th
   % order TF with 2 resonance peaks, than the index of the nat_freqs
   % vector found by the unique function would be 1,3,5, where 1 and 3 resonances 
   % and 5 is an aditional pole.Therefore, the logic of 
   % nat_freqs(idx(k)+1) == nat_freqs(idx(k)) is only respected by ressonance frequencies.

        if idx(k)+1 <= length(nat_freqs)  
            if nat_freqs(idx(k)+1) == nat_freqs(idx(k))
                unique_nat_freqs_shoulder = horzcat(unique_nat_freqs_shoulder,nat_freqs(idx(k))); % horzcat concatenates nat_freqs(idx(k)) into unique_nat_freqs_shoulder
                indices                   = horzcat(indices,idx(k)); % horzcat concatenates idx(k) into indices
            end
        end
    end
% 
%     for k = 1:length(idz) % idx are the indices of the poles that appers once.
%    
%    % Note that if the index is grather than the length of
%    % of the vector of poles, than it can not be a natural frequency because
%    % natural frequencies appears in pairs. EX: if you have a 5th
%    % order TF with 2 resonance peaks, than the index of the nat_freqs
%    % vector found by the unique function would be 1,3,5, where 1 and 3 resonances 
%    % and 5 is an aditional pole.Therefore, the logic of 
%    % nat_freqs(idx(k)+1) == nat_freqs(idx(k)) is only respected by ressonance frequencies.
% 
%         if idz(k)+1 <= length(zero_wn) 
%             zero_wn(idz(k)) 
%             zero_wn(idz(k)+1)
%             if zero_wn(idz(k)+1) == zero_wn(idz(k))
%                 unique_zero_wn_shoulder = horzcat(unique_zero_wn_shoulder,zero_wn(idz(k))); % horzcat concatenates nat_freqs(idx(k)) into unique_nat_freqs_shoulder
%                 indices_z               = horzcat(indices,idz(k)); % horzcat concatenates idx(k) into indices
%             end
%         end
%     end
    
    % Extracting the conjugate and nonconjugate zeros
    
%     [wn_index,zero_index] = gimme_index(zero_wn(i,:)); % wn_index is the complex conjugate index
%     z1                    = zero_wn(i,wn_index); % zero_index is the noncomplex conjugate index
    [wn_index,zero_index] = gimme_index(zeros_shoulder(i,:)); % wn_index is the complex conjugate index
    z1                    = zeros_shoulder(i,wn_index);    


    if length(z1)>1 % this if is necessary because we have 2 pairs of complex conjugate zeros sometimes
        cont_2_complex_conjugate     = horzcat(cont_2_complex_conjugate,i); % Poses we have two complex conjugate poles
        z_complex_conjugate_1        = horzcat(z_complex_conjugate_1,z1(1));
        z_complex_conjugate_2        = horzcat(z_complex_conjugate_2,z1(2));
    else
        z_complex_conjugate_1        = horzcat(z_complex_conjugate_1,z1(1));
        z_non_complex_conjugate(i,:) = zeros_shoulder(i,zero_index);
%         zeros_shoulder(i,zero_index)
%         zeros_shoulder(i,:)
%         disp(' ')
    end
    
    [~,pole_index]                        = gimme_index_pole(nat_freqs_shoulder(i,:));
    non_complex_conjugate_pole(i,:)       = nat_freqs_shoulder(i,pole_index);
    index_non_complex_conjugate_pole(i,:) = pole_index;
    
    [nat_freqs_shoulder_high(i),ind]     = min(unique_nat_freqs_shoulder); % [M,I] = min(A) returns the index of the minimal value in A as well as the value.  
    index_nat_freq_shoulder(i)           = indices(ind);

    [nat_freqs_shoulder_max(i),ind_max]  = max(unique_nat_freqs_shoulder); % [M,I] = min(A) returns the index of the minimal value in A as well as the value.  
    index_nat_freq_shoulder_max(i)       = indices(ind_max);


    if (nat_freqs_shoulder_high(i)/(2*pi)) > 90 && min(unique_nat_freqs_shoulder) ~= nat_freqs_shoulder_high(i)
        
        [nat_freqs_shoulder_high(i),ind] = min(unique_nat_freqs_shoulder);
        index_nat_freq_shoulder(i)       = indices(ind);

    end
end

% compute damping ratios
zeta_shoulder          = zeros(length(nat_freqs_shoulder),1);
zeta_shoulder_max      = zeros(length(nat_freqs_shoulder),1);
zeta_shoulder_5th_pole = zeros(length(nat_freqs_shoulder),1);

for i = 1:length(nat_freqs_shoulder)

    pole             = poles_shoulder(i,index_nat_freq_shoulder(i));
    wn               = abs(pole);
    sigma            = -real(pole);
    zeta_shoulder(i) = sigma/wn;

    pole_max             = poles_shoulder(i,index_nat_freq_shoulder_max(i));
    wn_max               = abs(pole_max);
    sigma_max            = -real(pole_max);
    zeta_shoulder_max(i) = sigma_max/wn_max;

    pole_5th                  = poles_shoulder(i,index_non_complex_conjugate_pole(i,:));
    wn_5th                    = abs(pole_5th);
    sigma_5th                 = -real(pole_5th);
    zeta_shoulder_5th_pole(i) = sigma_5th/wn_5th; % If all zeta is 1, this means the poles has no imaginary part

end

z_zeta_complex_conjugate_shoulder1     = zeros(length(nat_freqs_shoulder),1);
z_zeta_complex_conjugate_shoulder2     = zeros(length(nat_freqs_shoulder),1);
z_zeta_noncomplex_conjugate_shoulder1  = zeros(length(nat_freqs_shoulder),1);
z_zeta_noncomplex_conjugate_shoulder2  = zeros(length(nat_freqs_shoulder),1);

cont                                   = 0;
V_plot_z                               = V_rad_shoulder;
R_plot_z                               = R_m_shoulder;
R_plot_z(cont_2_complex_conjugate)     = []; % This R is just to plot the zeros
V_plot_z(cont_2_complex_conjugate)     = []; % This V is just to plot the zeros

for i = 1:length(zeros_shoulder)

    zero                                  = z_complex_conjugate_1(i);
    wn                                    = abs(zero);
    sigma                                 = -real(zero);
    z_zeta_complex_conjugate_shoulder1(i) = sigma/wn;

    if ismember(i,cont_2_complex_conjugate)
        
        cont                 = cont+1;
        zero                 = z_complex_conjugate_2(cont);
        wn_z2                = abs(zero);
        sigma_z2             = -real(zero);
        z_zeta_complex_conjugate_shoulder2(i) = sigma_z2/wn_z2;


    else

        zero                                     = z_non_complex_conjugate(i,1);
        wn                                       = abs(zero);
        sigma                                    = -real(zero);
        z_zeta_noncomplex_conjugate_shoulder1(i) = sigma/wn;

        zero                                     = z_non_complex_conjugate(i,2);
        wn                                       = abs(zero);
        sigma                                    = -real(zero);
        z_zeta_noncomplex_conjugate_shoulder2(i) = sigma/wn;

     end
      
end

z_wn_complex_conjugate_shoulder1 = abs(z_complex_conjugate_1')/(2*pi);
%save("Shoulder_zeros",'z_zeta_complex_conjugate_shoulder1','z_wn_complex_conjugate_shoulder1')

% figure(5)
% plot(J1_shoulder,nat_freqs_shoulder_high/(2*pi),'*','LineWidth',2)
% xlabel('Shoulder inertia [kg-m^2]')
% ylabel('Natural Frequency [Hz]')
% grid on
% title('Shoulder Joint Natural Frequencies')
% 
% figure(55)
% plot(J1_shoulder,zeta_shoulder,'*','LineWidth',2)
% xlabel('Shoulder inertia [kg-m^2]')
% ylabel('Damping Ratio [1]')
% grid on
% title('Shoulder Joint Damping Ratios')
% 
% figure(555)
% plot(J1_shoulder,2*zeta_shoulder.*nat_freqs_shoulder_high,'*','LineWidth',2)
% xlabel('Shoulder inertia [kg-m^2]')
% ylabel('2\zeta\omega_n')
% grid on
% title('Shoulder Joint Damping Coefficient')
figure(90)
plot3(V_rad_shoulder*(180/pi),R_m_shoulder*1000,abs(z_complex_conjugate_1)/(2*pi), '*','LineWidth',2)
xlabel('Angle from horizontal, V [deg]')
ylabel('Radius from base, R [mm]')
zlabel('Wn [Hz]')
grid on
title('First Complex conjugate zero - Shoulder')
zlim([20 30])

figure(91)
plot3(V_rad_shoulder*(180/pi),R_m_shoulder*1000,z_zeta_complex_conjugate_shoulder1, '*','LineWidth',2)
xlabel('Angle from horizontal, V [deg]')
ylabel('Radius from base, R [mm]')
zlabel('Damping ratio [-]')
grid on
title('Damping ratio of first Complex conjugate zero- Shoulder')
%zlim([0 0.3])

% z_plot                           = z_non_complex_conjugate(:,1);
% z_plot(cont_2_complex_conjugate) = [];
% figure(92)
% plot3(V_plot_z*(180/pi),R_plot_z*1000,abs(z_plot)/(2*pi), '*','LineWidth',2)
% xlabel('Angle from horizontal, V [deg]')
% ylabel('Radius from base, R [mm]')
% zlabel('Wn [Hz]')
% grid on
% title('First non-complex conjugate zero')
% zlim([0 1000])

% z_plot                           = z_non_complex_conjugate(:,2);
% z_plot(cont_2_complex_conjugate) = [];
% figure(93)
% plot3(V_plot_z*(180/pi),R_plot_z*1000,abs(z_plot)/(2*pi), '*','LineWidth',2)
% xlabel('Angle from horizontal, V [deg]')
% ylabel('Radius from base, R [mm]')
% zlabel('Wn [Hz]')
% grid on
% title('Second non-complex conjugate zero')
% zlim([0 50])


figure(9)
plot3(V_rad_shoulder*(180/pi),R_m_shoulder*1000,nat_freqs_shoulder_high/(2*pi), '*','LineWidth',2)
xlabel('Angle from horizontal, V [deg]')
ylabel('Radius from base, R [mm]')
zlabel('Natural frequency [Hz]')
grid on
title('Shoulder Joint Ressonance - First Pole')
%zlim([14 20])

figure(10)
plot3(V_rad_shoulder*(180/pi),R_m_shoulder*1000,zeta_shoulder, '*','LineWidth',2)
xlabel('Angle from horizontal, V [deg]')
ylabel('Radius from base, R [mm]')
zlabel('damping ration')
grid on
title('Shoulder Joint damping ration - First pole')


figure(11)
plot3(V_rad_shoulder*(180/pi),R_m_shoulder*1000,nat_freqs_shoulder_max/(2*pi), '*','LineWidth',2)
xlabel('Angle from horizontal, V [deg]')
ylabel('Radius from base, R [mm]')
zlabel('Natural frequency [Hz]')
grid on
title('Shoulder Joint Ressonance - Second Pole')
%zlim([20 35])
%xlim([0 80])
%ylim([200 550])

figure(12)
plot3(V_rad_shoulder*(180/pi),R_m_shoulder*1000,zeta_shoulder_max, '*','LineWidth',2)
xlabel('Angle from horizontal, V [deg]')
ylabel('Radius from base, R [mm]')
zlabel('damping ration')
grid on
title('Shoulder Joint damping ration - Second pole of shoulder FRF')

% figure(94)
% plot3(V_rad_shoulder*(180/pi),R_m_shoulder*1000,non_complex_conjugate_pole, '*','LineWidth',2)
% xlabel('Angle from horizontal, V [deg]')
% ylabel('Radius from base, R [mm]')
% zlabel('Wn[Hz]')
% grid on
% title('5th pole of shoulder FRF')
% zlim([0 200])

%% Edited on 12-Jun-2023
%%%%%%%%%%%%% Elbow joint
nat_freqs_elbow_high = zeros(length(nat_freqs_elbow),1);     % Firts poles of the system 
index_nat_freq_elbow = zeros(length(nat_freqs_elbow),1);

nat_freqs_elbow_max      = zeros(length(nat_freqs_elbow),1); % Second poles of the system 
index_nat_freq_elbow_max = zeros(length(nat_freqs_elbow),1);

% I noticed we normally have 1 non_complex_conjugate zeros and 1 gain
z_complex_conjugate_elbow_1    = zeros(length(nat_freqs_elbow),1);
z_non_complex_conjugate_elbow  = zeros(length(nat_freqs_elbow),1);

for i = 1:length(nat_freqs_elbow)
    nat_freqs = nat_freqs_elbow(i,:);
    [~,idx]   = unique(nat_freqs_elbow(i,:),'stable'); % this function identify unique values in the arrays.
    indices   = [];

    unique_nat_freqs_elbow  = [];
    unique_zero_wn_elbow    = []; % horzcat concatenates nat_freqs(idx(k)) into unique_nat_freqs_elbow
    indices_z               = [];
    

   for k = 1:length(idx) % idx are the indices of the poles that appers once.
   
   % Note that if the index is grather than the length of
   % of the vector of poles, than it can not be a natural frequency because
   % natural frequencies appears in pairs. EX: if you have a 5th
   % order TF with 2 resonance peaks, than the index of the nat_freqs
   % vector found by the unique function would be 1,3,5, where 1 and 3 resonances 
   % and 5 is an aditional pole.Therefore, the logic of 
   % nat_freqs(idx(k)+1) == nat_freqs(idx(k)) is only respected by ressonance frequencies.

        if idx(k)+1 <= length(nat_freqs)  
            if nat_freqs(idx(k)+1) == nat_freqs(idx(k))
                unique_nat_freqs_elbow = horzcat(unique_nat_freqs_elbow,nat_freqs(idx(k))); % horzcat concatenates nat_freqs(idx(k)) into unique_nat_freqs_shoulder
                indices                = horzcat(indices,idx(k)); % horzcat concatenates idx(k) into indices
            end
        end
    end
% 
%     for k = 1:length(idz) % idx are the indices of the poles that appers once.
%    
%    % Note that if the index is grather than the length of
%    % of the vector of poles, than it can not be a natural frequency because
%    % natural frequencies appears in pairs. EX: if you have a 5th
%    % order TF with 2 resonance peaks, than the index of the nat_freqs
%    % vector found by the unique function would be 1,3,5, where 1 and 3 resonances 
%    % and 5 is an aditional pole.Therefore, the logic of 
%    % nat_freqs(idx(k)+1) == nat_freqs(idx(k)) is only respected by ressonance frequencies.
% 
%         if idz(k)+1 <= length(zero_wn) 
%             zero_wn(idz(k)) 
%             zero_wn(idz(k)+1)
%             if zero_wn(idz(k)+1) == zero_wn(idz(k))
%                 unique_zero_wn_shoulder = horzcat(unique_zero_wn_shoulder,zero_wn(idz(k))); % horzcat concatenates nat_freqs(idx(k)) into unique_nat_freqs_shoulder
%                 indices_z               = horzcat(indices,idz(k)); % horzcat concatenates idx(k) into indices
%             end
%         end
%     end
    
    % Extracting the conjugate and nonconjugate zeros
    
%     [wn_index,zero_index] = gimme_index(zero_wn(i,:)); % wn_index is the complex conjugate index
%     z1                    = zero_wn(i,wn_index); % zero_index is the noncomplex conjugate index
    
    [wn_index,zero_index] = gimme_index(zeros_elbow(i,:)); % wn_index is the complex conjugate index
    z1                    = zeros_elbow(i,wn_index);    

%     z_complex_conjugate_elbow_1    = horzcat(z_complex_conjugate_1,z1(1));
    z_complex_conjugate_elbow_1(i,:)    = z1;
    z_non_complex_conjugate_elbow (i,:) = zeros_elbow(i,zero_index);
    
    [nat_freqs_elbow_high(i),ind]     = min(unique_nat_freqs_elbow); % [M,I] = min(A) returns the index of the minimal value in A as well as the value.  
    index_nat_freq_elbow(i)           = indices(ind);

    [nat_freqs_elbow_max(i),ind_max]  = max(unique_nat_freqs_elbow); % [M,I] = min(A) returns the index of the minimal value in A as well as the value.  
    index_nat_freq_elbow_max(i)       = indices(ind_max);


    if (nat_freqs_elbow_high(i)/(2*pi)) > 90 && min(unique_nat_freqs_elbow) ~= nat_freqs_elbow_high(i)
        
        [nat_freqs_elbow_high(i),ind] = min(unique_nat_freqs_elbow);
        index_nat_freq_elbow(i)       = indices(ind);

    end
end

% compute damping ratios
zeta_elbow          = zeros(length(nat_freqs_elbow),1);
zeta_elbow_max      = zeros(length(nat_freqs_elbow),1);

for i = 1:length(nat_freqs_elbow)

    pole             = poles_elbow(i,index_nat_freq_elbow(i));
    wn               = abs(pole);
    sigma            = -real(pole);
    zeta_elbow(i) = sigma/wn;

    pole_max             = poles_elbow(i,index_nat_freq_elbow_max(i));
    wn_max               = abs(pole_max);
    sigma_max            = -real(pole_max);
    zeta_elbow_max(i)    = sigma_max/wn_max;

end

z_zeta_complex_conjugate_elbow1     = zeros(length(nat_freqs_elbow),1);

V_plot_z                            = V_rad_elbow;
R_plot_z                            = R_m_elbow;

for i = 1:length(zeros_elbow)

    zero                                  = z_complex_conjugate_elbow_1(i);
    wn                                    = abs(zero);
    sigma                                 = -real(zero);
    z_zeta_complex_conjugate_elbow1(i) = sigma/wn;
      
end

figure(900)
plot3(V_rad_elbow*(180/pi),R_m_elbow*1000,abs(z_complex_conjugate_elbow_1)/(2*pi), '*','LineWidth',2)
xlabel('Angle from horizontal, V [deg]')
ylabel('Radius from base, R [mm]')
zlabel('Wn [Hz]')
grid on
title('First Complex conjugate zero - Elbow')
%zlim([20 30])

figure(901)
plot3(V_rad_elbow*(180/pi),R_m_elbow*1000,z_zeta_complex_conjugate_elbow1, '*','LineWidth',2)
xlabel('Angle from horizontal, V [deg]')
ylabel('Radius from base, R [mm]')
zlabel('Damping ratio [-]')
grid on
title('Damping ratio of first Complex conjugate zero - Elbow')
%zlim([0 0.3])

figure(902)
plot3(V_rad_elbow*(180/pi),R_m_elbow*1000,nat_freqs_elbow_high/(2*pi), '*','LineWidth',2)
xlabel('Angle from horizontal, V [deg]')
ylabel('Radius from base, R [mm]')
zlabel('Natural frequency [Hz]')
grid on
title('elbow Joint Ressonance - First Pole')
%zlim([14 20])

figure(903)
plot3(V_rad_elbow*(180/pi),R_m_elbow*1000,zeta_elbow, '*','LineWidth',2)
xlabel('Angle from horizontal, V [deg]')
ylabel('Radius from base, R [mm]')
zlabel('damping ration')
grid on
title('elbow Joint damping ration - First pole')


figure(904)
plot3(V_rad_elbow*(180/pi),R_m_elbow*1000,nat_freqs_elbow_max/(2*pi), '*','LineWidth',2)
xlabel('Angle from horizontal, V [deg]')
ylabel('Radius from base, R [mm]')
zlabel('Natural frequency [Hz]')
grid on
title('elbow Joint Ressonance - Second Pole')

figure(9044)
plot3(V_rad_elbow*(180/pi),J1_shoulder,nat_freqs_elbow_max/(2*pi), '*','LineWidth',2)
xlabel('Angle from horizontal, V [deg]')
ylabel('J1 - shoulder inertia')
zlabel('Natural frequency [Hz]')
grid on
title('elbow Joint Ressonance - Second Pole')

figure(9045)
plot3(V_rad_elbow*(180/pi),J2_elbow,nat_freqs_elbow_max/(2*pi), '*','LineWidth',2)
xlabel('Angle from horizontal, V [deg]')
ylabel('J2 - elbow inertia')
zlabel('Natural frequency [Hz]')
grid on
title('elbow Joint Ressonance - Second Pole')


%zlim([20 35])
%xlim([0 80])
%ylim([200 550])

figure(905)
plot3(V_rad_elbow*(180/pi),R_m_elbow*1000,zeta_elbow_max, '*','LineWidth',2)
xlabel('Angle from horizontal, V [deg]')
ylabel('Radius from base, R [mm]')
zlabel('damping ration')
grid on
title('elbow Joint damping ration - Second pole of elbow FRF')


% V_idx = find(abs(V_rad_base-0.3927) < 0.001); % This part is used o plot a selected line 
% 
% figure(13)
% plot3(V_rad_base(V_idx)*(180/pi),R_m_base(V_idx)*1000,nat_freqs_base_high(V_idx)/(2*pi), '*','LineWidth',2)
% xlabel('Angle from horizontal, V [deg]')
% ylabel('Radius from base, R [mm]')
% zlabel('Natural frequency [Hz]')
% grid on
% title('Base Joint Natural Frequencies (V = 22.5^{\circ})')
% 
% R_idx = find(abs(R_m_shoulder-0.470883) < 0.01);
% 
% figure(14)
% plot3(V_rad_shoulder*(180/pi),R_m_shoulder*1000,J1_shoulder, '*','LineWidth',2)
% xlabel('Angle from horizontal, V [deg]')
% ylabel('Radius from base, R [mm]')
% zlabel('Shoulder Inertia [kg-m^2]')
% grid on
% title('Shoulder Joint Inertia ')
% 
% 
% figure(15)
% plot3(V_rad_shoulder(R_idx)*(180/pi),R_m_shoulder(R_idx)*1000,nat_freqs_shoulder_high(R_idx)/(2*pi), '*','LineWidth',2)
% xlabel('Angle from horizontal, V [deg]')
% ylabel('Radius from base, R [mm]')
% zlabel('Natural frequency [Hz]')
% grid on
% title('Shoulder Joint Natural Frequencies (R = 470.83 mm)')
%%
% check_idx = 55:60;
% figure(111)
% subplot(2,1,1)
% semilogx(freq_range,mag2db(abs(H_B(:,check_idx))),'LineWidth',2)
% hold on
% semilogx(freq_range_fit,mag2db(abs(H_B_fit(:,check_idx))),'--','LineWidth',2)
% ylabel('Mag [dB]')
% grid on
% xlim([2 60])
% title('Base Joint FRFs (Measured)')
% % legend('[0, 0, 0, 0, 0, 0]', '[0, -\pi/2, 0, -\pi/2, 0, 0]', '[\pi/2, -\pi/2, 0, -\pi/2, 0, 0]','[\pi/2, -\pi/2, 5\pi/6, -\pi/2, 0, 0]', 'Location', 'southwest')
% 
% subplot(2,1,2)
% semilogx(freq_range,unwrap(angle(H_B(:,check_idx)))/pi*180,'LineWidth',2)
% hold on
% semilogx(freq_range_fit,unwrap(angle(H_B_fit(:,check_idx)))/pi*180,'--','LineWidth',2)
% xlabel('Frequency [Hz]')
% ylabel('Phase [deg]')
% grid on
% xlim([2 60])

%%
% check_Sidx = 19;
% figure(161)
% subplot(2,1,1)
% semilogx(freq_range,mag2db(abs(H_S(:,check_Sidx))),'LineWidth',2)
% hold on
% semilogx(freq_range_fit,mag2db(abs(H_S_fit(:,check_Sidx))),'--','LineWidth',2)
% ylabel('Mag [dB]')
% grid on
% xlim([2 60])
% title('Shoulder Joint FRFs (Measured)')
% % legend('[0, 0, 0, 0, 0, 0]', '[0, -\pi/2, 0, -\pi/2, 0, 0]', '[\pi/2, -\pi/2, 0, -\pi/2, 0, 0]','[\pi/2, -\pi/2, 5\pi/6, -\pi/2, 0, 0]', 'Location', 'southwest')
% 
% subplot(2,1,2)
% semilogx(freq_range,unwrap(angle(H_S(:,check_Sidx)))/pi*180,'LineWidth',2)
% hold on
% semilogx(freq_range_fit,unwrap(angle(H_S_fit(:,check_Sidx)))/pi*180,'--','LineWidth',2)
% xlabel('Frequency [Hz]')
% ylabel('Phase [deg]')
% grid on
% xlim([2 60])

% save('R_V_NatFreq_Damping_16_May_short_BW.mat','V_rad_base','R_m_base','nat_freqs_base_high','zeta_base',...
%     'second_pole_index','zeta_base_max','nat_freqs_base_max',...
%     'V_rad_shoulder','R_m_shoulder','nat_freqs_shoulder_high','zeta_shoulder',...
%      'zeta_shoulder_max','nat_freqs_shoulder_max')

%save('R_V_NatFreq_Damping_14_jun_base.mat','V_rad_base','R_m_base','nat_freqs_base_high',...
%    'nat_freqs_base_max','zeta_base','zeta_base_max','z_complex_conjugate_1','z_zeta_complex_conjugate_shoulder1')

%save('R_V_NatFreq_Damping_14_jun_shoulder.mat','V_rad_shoulder','R_m_shoulder','nat_freqs_shoulder_high',...
%    'nat_freqs_shoulder_max','zeta_shoulder','zeta_shoulder_max','z_complex_conjugate_1','z_zeta_complex_conjugate_shoulder1')

%save('R_V_NatFreq_Damping_14_jun_elbow.mat','V_rad_elbow','R_m_elbow','z_complex_conjugate_elbow_1',...
%    'nat_freqs_elbow_high','nat_freqs_elbow_max','zeta_elbow','zeta_elbow_max','z_zeta_complex_conjugate_elbow1')


%% Computing stiffness constants with least squares
% -- base stiffness --
% A_base      = ones(size(nat_freqs_base,1),1);
% A_base(:,2) = J0_base;
% 
% b_base = J0_base.*nat_freqs_base_high.^2;
% 
% k_x_base = pinv(A_base)*b_base;
% 
% k0_base = k_x_base(1);
% k1_base = k_x_base(2);
% 
% J0_base_line                 = linspace(min(J0_base),max(J0_base),100);
% sim_nat_freqs_base_high      = sqrt((k0_base + k1_base*J0_base)./J0_base);
% sim_nat_freqs_base_high_line = sqrt((k0_base + k1_base*J0_base_line)./J0_base_line);
% 
% % -- base stiffness with shoulder inertia --
% A_base = ones(size(nat_freqs_base,1),1);
% A_base(:,2) = J1_base;
% 
% b_base = J1_base.*nat_freqs_base_high.^2;
% 
% k_x_base = pinv(A_base)*b_base;
% 
% k0_base = k_x_base(1);
% k1_base = k_x_base(2);
% 
% J1_base_line = linspace(min(J1_base),max(J1_base),100);
% sim_nat_freqs_base_high_J1 = sqrt((k0_base + k1_base*J1_base)./J1_base);
% sim_nat_freqs_base_high_J1_line = sqrt((k0_base + k1_base*J1_base_line)./J1_base_line);

% -- shoulder stiffness -- Fist pole
A_shoulder = ones(size(nat_freqs_shoulder,1),1);
A_shoulder(:,2) = J1_shoulder;

b_shoulder = J1_shoulder.*nat_freqs_shoulder_high.^2;

k_x_shoulder = pinv(A_shoulder)*b_shoulder;

k0_shoulder = k_x_shoulder(1);
k1_shoulder = k_x_shoulder(2);

J1_shoulder_line                 = linspace(min(J1_shoulder),max(J1_shoulder),100);
sim_nat_freqs_shoulder_high      = sqrt((k0_shoulder + k1_shoulder*J1_shoulder)./J1_shoulder);
sim_nat_freqs_shoulder_high_line = sqrt((k0_shoulder + k1_shoulder*J1_shoulder_line)./J1_shoulder_line);

% -- shoulder stiffness -- Second pole
A_shoulder_max = ones(size(nat_freqs_shoulder,1),1);
A_shoulder_max(:,2) = J1_shoulder;

b_shoulder_max = J1_shoulder.*nat_freqs_shoulder_max.^2;

k_x_shoulder = pinv(A_shoulder_max)*b_shoulder_max;

k0_shoulder = k_x_shoulder(1);
k1_shoulder = k_x_shoulder(2);

sim_nat_freqs_shoulder_max      = sqrt((k0_shoulder + k1_shoulder*J1_shoulder)./J1_shoulder);
sim_nat_freqs_shoulder_max_line = sqrt((k0_shoulder + k1_shoulder*J1_shoulder_line)./J1_shoulder_line);

% % -- plot comparisons --
% figure(30)
% plot(J0_base,nat_freqs_base_high/(2*pi),'*','LineWidth',2)
% hold on
% plot(J0_base,sim_nat_freqs_base_high/(2*pi),'*','LineWidth',2)
% xlabel('Base inertia [kg-m^2]')
% ylabel('Natural Frequency [Hz]')
% grid on
% title('Base Joint Natural Frequencies')
% legend('Measured', 'Fit')

% figure(40)
% plot(J0_base,nat_freqs_base_high/(2*pi),'*','LineWidth',2)
% hold on
% plot(J0_base_line,sim_nat_freqs_base_high_line/(2*pi),'LineWidth',2)
% xlabel('Base inertia [kg-m^2]')
% ylabel('Natural Frequency [Hz]')
% grid on
% title('Base Joint Natural Frequencies')
% legend('Measured', 'Fit')

% figure(31)
% plot(J1_base,nat_freqs_base_high/(2*pi),'*','LineWidth',2)
% hold on
% plot(J1_base,sim_nat_freqs_base_high_J1/(2*pi),'*','LineWidth',2)
% xlabel('Shoulder inertia [kg-m^2]')
% ylabel('Natural Frequency [Hz]')
% grid on
% title('Base Joint Natural Frequencies')
% legend('Measured', 'Fit')
% % ylim([0,30])
% % xlim([0,10])

% figure(41)
% plot(J1_base,nat_freqs_base_high/(2*pi),'*','LineWidth',2)
% hold on
% plot(J1_base_line,sim_nat_freqs_base_high_J1_line/(2*pi),'LineWidth',2)
% xlabel('Shoulder inertia [kg-m^2]')
% ylabel('Natural Frequency [Hz]')
% grid on
% title('Base Joint Natural Frequencies')
% legend('Measured', 'Fit')

figure(50)
plot(J1_shoulder,nat_freqs_shoulder_high/(2*pi),'*','LineWidth',2)
hold on
plot(J1_shoulder,sim_nat_freqs_shoulder_high/(2*pi),'*','LineWidth',2)
xlabel('Shoulder inertia [kg-m^2]')
ylabel('Natural Frequency [Hz]')
grid on
title('Shoulder Joint - First pole')
legend('Measured', 'Fit')

figure(51)
plot(J1_shoulder,nat_freqs_shoulder_max/(2*pi),'*','LineWidth',2)
hold on
plot(J1_shoulder,sim_nat_freqs_shoulder_max/(2*pi),'*','LineWidth',2)
xlabel('Shoulder inertia [kg-m^2]')
ylabel('Natural Frequency [Hz]')
grid on
title('Shoulder Joint - Second pole')
legend('Measured', 'Fit')

index_v = find(V_rad_shoulder*180/pi<80);
figure(52)
plot(J1_shoulder(index_v),nat_freqs_shoulder_max(index_v)/(2*pi),'*','LineWidth',2)
hold on
plot(J1_shoulder(index_v),sim_nat_freqs_shoulder_max(index_v)/(2*pi),'*','LineWidth',2)
xlabel('Shoulder inertia [kg-m^2]')
ylabel('Natural Frequency [Hz]')
grid on
title('Shoulder Joint - Second pole - Removing V>80')
legend('Measured', 'Fit')

figure(55563)
plot(R_m_shoulder*1000,nat_freqs_shoulder_max/(2*pi), '*','LineWidth',2);hold on
plot(R_m_shoulder*1000,sim_nat_freqs_shoulder_max/(2*pi),'*','LineWidth',2)
xlabel('Radius from base, R [mm]')
ylabel('Natural frequency [Hz]')
grid on
title('Shoulder Joint Ressonance - Second Pole')
legend('Measured', 'Fit')
%zlim([20 35])

% figure(51)
% plot(J1_shoulder,nat_freqs_shoulder_high/(2*pi),'*','LineWidth',2)
% hold on
% plot(J1_shoulder_line,sim_nat_freqs_shoulder_high_line/(2*pi),'LineWidth',2)
% xlabel('Shoulder inertia [kg-m^2]')
% ylabel('Natural Frequency [Hz]')
% grid on
% title('Shoulder Joint Natural Frequencies')
% legend('Measured', 'Fit')

% figure(303)
% plot3(J0_base,V_rad_base*(180/pi),nat_freqs_base_high/(2*pi),'*','LineWidth',2)
% xlabel('Base inertia [kg-m^2]')
% ylabel('Angle from horizontal, V [deg]')
% zlabel('Natural Frequency [Hz]')
% grid on
% title('Measured Base Joint Natural Frequencies')
% 
% figure(304)
% plot3(V_rad_base*(180/pi),R_m_base*1000,J0_base,'*','LineWidth',2)
% xlabel('Angle from horizontal, V [deg]')
% ylabel('Radius from base, R [mm]')
% zlabel(' joint na inertia [kg-m^2]')
% grid on
% title('Base Joint Inertias')
% 
% figure(305)
% plot3(J1_base,V_rad_base*(180/pi),nat_freqs_base_high/(2*pi),'*','LineWidth',2)
% xlabel('Shoulder inertia [kg-m^2]')
% ylabel('Angle from horizontal, V [deg]')
% zlabel('Natural Frequency [Hz]')
% grid on
% title('Measured Base Joint Natural Frequencies')

%% Fit the damping constants
% -- base damping constant --
% damp_const_base = @(c) (c(1) + c(2)*sqrt(J0_base) + c(3)*J0_base).*(1+c(4)*J0_base) - 2*zeta_base.*nat_freqs_base_high.*J0_base;
%damp_const_base = @(c) (c(1) + c(2)*sqrt(J1_base) + c(3)*J1_base).*(1+c(4)*J1_base) - 2*zeta_base.*nat_freqs_base_high.*J1_base;
% 
% 
% rng('default')
% c_init_base = rand(4,1)*40; % initial value for nonlinear optimization
% 
% c_base = lsqnonlin(damp_const_base,c_init_base);
% 
% c0_base = c_base(1);
% c1_base = c_base(2);
% c2_base = c_base(3);
% c_tau_base = c_base(4);
% 
% % C_base = (c0_base + c1_base*sqrt(J0_base) + c2_base*J0_base).*(1+c_tau_base*J0_base);
% % C_base_line = (c0_base + c1_base*sqrt(J0_base_line) + c2_base*J0_base_line).*(1+c_tau_base*J0_base_line);
% 
% C_base = (c0_base + c1_base*sqrt(J1_base) + c2_base*J1_base).*(1+c_tau_base*J1_base);
% C_base_line = (c0_base + c1_base*sqrt(J1_base_line) + c2_base*J1_base_line).*(1+c_tau_base*J1_base_line);
% 
% % zeta_base_fit = C_base./(2*sim_nat_freqs_base_high.*J0_base);
% zeta_base_fit = C_base./(2*sim_nat_freqs_base_high.*J1_base);


% -- shoulder damping constant --
damp_const_shoulder = @(c) (c(1) + c(2)*sqrt(J1_shoulder) + c(3)*J1_shoulder).*(1+c(4)*J1_shoulder) - 2*zeta_shoulder.*nat_freqs_shoulder_high.*J1_shoulder;

c0_shoulder = rand(4,1)*50; % initial value for nonlinear optimization

c_shoulder = lsqnonlin(damp_const_shoulder,c0_shoulder);

c0_shoulder    = c_shoulder(1);
c1_shoulder    = c_shoulder(2);
c2_shoulder    = c_shoulder(3);
c_tau_shoulder = c_shoulder(4);

C_shoulder        = (c0_shoulder + c1_shoulder*sqrt(J1_shoulder) + c2_shoulder*J1_shoulder).*(1+c_tau_shoulder*J1_shoulder);
C_shoulder_line   = (c0_shoulder + c1_shoulder*sqrt(J1_shoulder_line) + c2_shoulder*J1_shoulder_line).*(1+c_tau_shoulder*J1_shoulder_line);

zeta_shoulder_fit = C_shoulder./(2*sim_nat_freqs_shoulder_high.*J1_shoulder);
% -- plot comparisons --
% figure(3330)
% plot(J0_base,2*zeta_base.*nat_freqs_base_high.*J0_base,'*','LineWidth',2)
% hold on
% plot(J0_base,C_base,'*','LineWidth',2)
% xlabel('Base inertia [kg-m^2]')
% ylabel('Damping coeff., c [kg-m^2/s]')
% grid on
% title('Base Joint Damping Coefficient')
% legend('Measured', 'Fit')

% figure(3331)
% plot(J0_base,2*zeta_base.*nat_freqs_base_high.*J0_base,'*','LineWidth',2)
% hold on
% plot(J0_base_line,C_base_line,'LineWidth',2)
% xlabel('Base inertia [kg-m^2]')
% ylabel('Damping coeff., c [kg-m^2/s]')
% grid on
% title('Base Joint Damping Coefficient')
% legend('Measured', 'Fit')

% figure(3332)
% plot(J0_base,zeta_base,'*','LineWidth',2)
% hold on
% plot(J0_base,zeta_base_fit,'*','LineWidth',2)
% xlabel('Base inertia [kg-m^2]')
% ylabel('Damping ratio, \zeta [1]')
% grid on
% title('Base Joint Damping Ratio')
% legend('Measured', 'Fit')
% ylim([0 0.7])

% figure(3333)
% plot(J1_base,2*zeta_base.*nat_freqs_base_high.*J1_base,'*','LineWidth',2)
% hold on
% plot(J1_base_line,C_base_line,'LineWidth',2)
% xlabel('Shoulder inertia [kg-m^2]')
% ylabel('Damping coeff., c [kg-m^2/s]')
% grid on
% title('Base Joint Damping Coefficient')
% legend('Measured', 'Fit')
% 
% figure(3334)
% plot(J1_base,zeta_base,'*','LineWidth',2)
% hold on
% plot(J1_base,zeta_base_fit,'*','LineWidth',2)
% xlabel('Shoulder inertia [kg-m^2]')
% ylabel('Damping ratio, \zeta [1]')
% grid on
% title('Base Joint Damping Ratio')
% legend('Measured', 'Fit')
% ylim([0 0.7])
% 

figure(5550)
plot(J1_shoulder,2*zeta_shoulder.*nat_freqs_shoulder_high.*J1_shoulder,'*','LineWidth',2)
hold on
plot(J1_shoulder,C_shoulder,'*','LineWidth',2)
xlabel('Shoulder inertia [kg-m^2]')
ylabel('Damping coeff., c [kg-m^2/s]')
grid on
title('Shoulder Joint Damping Coefficient')
legend('Measured', 'Fit')

figure(5551)
plot(J1_shoulder,2*zeta_shoulder.*nat_freqs_shoulder_high.*J1_shoulder,'*','LineWidth',2)
hold on
plot(J1_shoulder_line,C_shoulder_line,'LineWidth',2)
xlabel('Shoulder inertia [kg-m^2]')
ylabel('Damping coeff., c [kg-m^2/s]')
grid on
title('Shoulder Joint Damping Coefficient')
legend('Measured', 'Fit')

figure(5552)
plot(J1_shoulder,zeta_shoulder,'*','LineWidth',2)
hold on
plot(J1_shoulder,zeta_shoulder_fit,'*','LineWidth',2)
xlabel('Shoulder inertia [kg-m^2]')
ylabel('Damping ratio, \zeta [1]')
grid on
title('Shoulder Joint Damping Ratio')
legend('Measured', 'Fit')
ylim([0 0.7])

%% Using the electrical transfer functions to fit the predictions
% % base 
% test_idx = 4;
% s = tf('s');
% max_err_sum = 10^15; % arbitrarily high number to start
% for i = test_idx
%     sys = zpk(base_transfer_functions(i));
%     [z,p,k] = zpkdata(sys);
%     error_sum = 0;
%     remainder_idx = setdiff(1:length(p{1,1}),[index_nat_freq_base(i) index_nat_freq_base(i)+1]);
% 
%     for j = 1:length(nat_freqs_base)
%         c_pred   = (c0_base + c1_base*sqrt(J1_base(j)) + c2_base*J1_base(j)).*(1+c_tau_base*J1_base(j));
%         k_pred   = k0_base + k1_base*J1_base(j);
%         frf_pred = (c_pred/J1_base(j)*s + k_pred/J1_base(j)) / (s^2 + c_pred/J1_base(j)*s + k_pred/J1_base(j));
% 
%         [p_pred,~] = pzmap(frf_pred);
%         scale_nf = nat_freqs_base_high(i)^2 / abs(p_pred(1))^2;  
% 
%         sys_pred = zpk(z{1,1},[p{1,1}(remainder_idx); p_pred],k/scale_nf);
%         H_B_pred = freqresp(sys_pred,freq_range_fit,'Hz');
%         H_diff = abs(H_B_fit(:,j) - H_B_pred(:));
%         error_sum = error_sum + sum(H_diff);
%     end
% 
% %     if error_sum <= max_err_sum
%     if i == test_idx
%         max_err_sum   = error_sum;
%         sys_elec_base = zpk(z{1,1},p{1,1}(remainder_idx),k);
%         nat_freq_from_elec_base = nat_freqs_base_high(i);
%         tf_idx_used = i;
%         break
%     end
% end
% 
% %% Plot predictions and fitted transfer functions to see difference - base joint
% % close all
% H_B_pred_save = zeros(size(H_B_fit,1),length(base_transfer_functions));
% [z,p,g] = zpkdata(sys_elec_base);
% for k = 1:length(base_transfer_functions)
%     c_pred   = (c0_base + c1_base*sqrt(J1_base(k)) + c2_base*J1_base(k)).*(1+c_tau_base*J1_base(k));
%     k_pred   = k0_base + k1_base*J1_base(k);
%     frf_pred = (c_pred/J1_base(k)*s + k_pred/J1_base(k)) / (s^2 + c_pred/J1_base(k)*s + k_pred/J1_base(k));
% 
%     [p_pred,~] = pzmap(frf_pred);
%     scale_nf   = nat_freq_from_elec_base^2 / abs(p_pred(1))^2;
% 
%     sys_pred = zpk(z{1,1},[p{1,1}; p_pred],g/scale_nf);
%     H_B_pred = freqresp(sys_pred,freq_range_fit,'Hz');
%     H_B_pred_save(:,k) = H_B_pred(1,:,:);
% 
%     figure(121)
%     subplot(2,1,1)
%     c1=semilogx(freq_range,mag2db(abs(H_B(:,k))),'LineWidth',2);
%     hold on
%     c2=semilogx(freq_range_fit,mag2db(abs(H_B_pred(:))),'k--','LineWidth',2);
%     c3=text(3, -40, ['q_1: ', num2str(round(joint_positions_base(k,1),2)), ', q_2: ', num2str(round(joint_positions_base(k,2),2)),...
%         ', q_3: ', num2str(round(joint_positions_base(k,3),2)), ', q_4: ', num2str(round(joint_positions_base(k,4),2)),... 
%         ', q_5: ', num2str(round(joint_positions_base(k,5),2)), ', q_6: ', num2str(round(joint_positions_base(k,6),2)),', V: ',...
%         num2str(round(V_rad_base(k),2)),', R: ', num2str(round(R_m_base(k),2))]);
%     ylabel('Mag [dB]')
%     grid on
%     xlim([2 60])
%     title('Base Joint FRFs (Fits & Predictions)')
%     hold off
% 
%     subplot(2,1,2)
%     h1=semilogx(freq_range,unwrap(angle(H_B(:,k)))/pi*180,'LineWidth',2);
%     hold on
%     h2=semilogx(freq_range_fit,unwrap(angle(H_B_pred(:)))/pi*180,'k--','LineWidth',2);
%     xlabel('Frequency [Hz]')
%     ylabel('Phase [deg]')
%     grid on
%     xlim([2 60])
%     hold off
%     pause(0.001);
% 
%     if k >= 1
%         delete(c3)
%     end
% end
% %%
% figure(150)
% subplot(2,1,1)
% semilogx(freq_range_fit,mag2db(abs(H_B_pred_save)),'LineWidth',2)
% ylabel('Mag [dB]')
% grid on
% xlim([2 60])
% title('Base Joint FRFs (Least Squares Prediction)')
% % legend('[0, 0, 0, 0, 0, 0]', '[0, -\pi/2, 0, -\pi/2, 0, 0]', '[\pi/2, -\pi/2, 0, -\pi/2, 0, 0]','[\pi/2, -\pi/2, 5\pi/6, -\pi/2, 0, 0]', 'Location', 'southwest')
% 
% subplot(2,1,2)
% semilogx(freq_range_fit,unwrap(angle(H_B_pred_save))/pi*180,'LineWidth',2)
% xlabel('Frequency [Hz]')
% ylabel('Phase [deg]')
% grid on
% xlim([2 60])
%%
% check_idx = tf_idx_used;
% c_pred = (c0_base + c1_base*sqrt(J1_base(check_idx)) + c2_base*J1_base(check_idx)).*(1+c_tau_base*J1_base(check_idx));
% k_pred = k0_base + k1_base*J1_base(check_idx);
% frf_pred = (c_pred/J1_base(check_idx)*s + k_pred/J1_base(check_idx)) / (s^2 + c_pred/J1_base(check_idx)*s + k_pred/J1_base(check_idx));
% 
% [p_pred,~] = pzmap(frf_pred);
% scale_nf = nat_freq_from_elec_base^2 / abs(p_pred(1))^2;
% 
% sys_pred = zpk(z{1,1},[p{1,1}; p_pred],g/scale_nf);
% H_B_pred = freqresp(sys_pred,freq_range_fit,'Hz');
% 
% figure(1111)
% subplot(2,1,1)
% semilogx(freq_range_fit,mag2db(abs(H_B_fit(:,check_idx))),'LineWidth',2)
% hold on
% semilogx(freq_range_fit,mag2db(abs(H_B_pred(:))),'k--','LineWidth',2);
% ylabel('Mag [dB]')
% grid on
% xlim([2 60])
% title('Base Joint FRFs (Measured)')
% % legend('[0, 0, 0, 0, 0, 0]', '[0, -\pi/2, 0, -\pi/2, 0, 0]', '[\pi/2, -\pi/2, 0, -\pi/2, 0, 0]','[\pi/2, -\pi/2, 5\pi/6, -\pi/2, 0, 0]', 'Location', 'southwest')
% 
% subplot(2,1,2)
% semilogx(freq_range_fit,unwrap(angle(H_B_fit(:,check_idx)))/pi*180,'LineWidth',2)
% hold on
% semilogx(freq_range_fit,unwrap(angle(H_B_pred(:)))/pi*180,'k--','LineWidth',2);
% xlabel('Frequency [Hz]')
% ylabel('Phase [deg]')
% grid on
% xlim([2 60])
%% Save models
% [z_elec_base,p_elec_base,g_elec_base] = zpkdata(sys_elec_base);
% z_elec_base = z_elec_base{1};
% p_elec_base = p_elec_base{1};

% save(['base_damping_stiffness_params_',date_str], 'k0_base', 'k1_base', 'c0_base',...
%     'c1_base', 'c2_base', 'c_tau_base', 'H_B', 'H_B_fit', 'freq_range',...
%     'freq_range_fit', 'z_elec_base','p_elec_base','g_elec_base','nat_freq_from_elec_base',...
%     'J0_base','J1_base','joint_positions_base');
%% Using the electrical transfer functions to fit the predictions
% Shoulder 
test_idx = 10;
s = tf('s');
max_err_sum = 10^15; % arbitrarily high number to start
for i = test_idx
    sys       = zpk(shoulder_transfer_functions(i));
    [z,p,k]   = zpkdata(sys);
    error_sum = 0;
    remainder_idx = setdiff(1:length(p{1,1}),[index_nat_freq_shoulder(i) index_nat_freq_shoulder(i)+1]);

    for j = 1:length(nat_freqs_shoulder)
        c_pred   = (c0_shoulder + c1_shoulder*sqrt(J1_shoulder(j)) + c2_shoulder*J1_shoulder(j)).*(1+c_tau_shoulder*J1_shoulder(j));
        k_pred   = k0_shoulder + k1_shoulder*J1_shoulder(j);
        frf_pred = (c_pred/J1_shoulder(j)*s + k_pred/J1_shoulder(j)) / (s^2 + c_pred/J1_shoulder(j)*s + k_pred/J1_shoulder(j));

        [p_pred,~] = pzmap(frf_pred);
        scale_nf   = nat_freqs_shoulder_high(i)^2 / abs(p_pred(1))^2;  

        sys_pred  = zpk(z{1,1},[p{1,1}(remainder_idx); p_pred],k/scale_nf);
        H_S_pred  = freqresp(sys_pred,freq_range_fit,'Hz');
        H_diff    = abs(H_S_fit(:,j) - H_S_pred(:));
        error_sum = error_sum + sum(H_diff);
    end

%     if error_sum <= max_err_sum
    if i == test_idx
        max_err_sum = error_sum;
        sys_elec_shoulder = zpk(z{1,1},p{1,1}(remainder_idx),k);
        nat_freq_from_elec_shoulder = nat_freqs_shoulder_high(i);
        tf_idx_used = i;
        break
    end
end

%% Plot predictions and fitted transfer functions to see difference 

%% shoulder joint
% close all
H_S_pred_save = zeros(size(H_S_fit,1),length(shoulder_transfer_functions));
[z,p,g]       = zpkdata(sys_elec_shoulder);

gain_pz       = zeros(length(shoulder_transfer_functions),1); % TF correction to make sure we will start from 0 db
cont          = 0;
H_S_pred      = zeros(size(H_S_fit));
G_pz_nogain   = tf(zeros(length(shoulder_transfer_functions),1)); 
G_pz_save     = tf(zeros(length(shoulder_transfer_functions),1));

for k = 1:length(shoulder_transfer_functions)
%     c_pred    = (c0_shoulder + c1_shoulder*sqrt(J1_shoulder(k)) + c2_shoulder*J1_shoulder(k)).*(1+c_tau_shoulder*J1_shoulder(k));
%     k_pred    = k0_shoulder + k1_shoulder*J1_shoulder(k);
%     frf_pred  = (c_pred/J1_shoulder(k)*s + k_pred/J1_shoulder(k)) / (s^2 + c_pred/J1_shoulder(k)*s + k_pred/J1_shoulder(k));


    % Define the natural poles and zeros
    s           = tf([1 0],[1]);
    img         = sqrt(-1);

    % Pole/zero model = -zeta*wn+wn*sqrt(1-zeta^2)
    % Complex conjugate zero
    zeta        = z_zeta_complex_conjugate_shoulder1(k);
    wn          = abs(z_complex_conjugate_1(k));
    z_1         = -zeta*wn + wn*sqrt(1-zeta^2)*img;  
    z_1_conj    = -zeta*wn - wn*sqrt(1-zeta^2)*img;
    
    % Complex conjugate pole 1 
    zeta        = zeta_shoulder(k);
    wn          = abs(poles_shoulder(k,index_nat_freq_shoulder(k)));
    p_1         = -zeta*wn + wn*sqrt(1-zeta^2)*img;  
    p_1_conj    = -zeta*wn - wn*sqrt(1-zeta^2)*img;
    
    % Complex conjugate pole 2 
    zeta        = zeta_shoulder_max(k);
    wn          = abs(poles_shoulder(k,index_nat_freq_shoulder_max(k)));
    p_2         = -zeta*wn + wn*sqrt(1-zeta^2)*img;  
    p_2_conj    = -zeta*wn - wn*sqrt(1-zeta^2)*img;
    
    if ismember(k,cont_2_complex_conjugate)
         
        % Complex conjugate zero 2
         cont        = cont+1;
         zeta        = z_zeta_complex_conjugate_shoulder2(k);
         wn          = abs(z_complex_conjugate_2(cont));
         z_2         = -zeta*wn + wn*sqrt(1-zeta^2)*img;  
         z_2_conj    = -zeta*wn - wn*sqrt(1-zeta^2)*img;
         G_pz        =  (s-z_1)*(s-z_1_conj)*(s-z_2)*(s-z_2_conj)/((s-p_1)*(s-p_1_conj)*(s-p_2)*(s-p_2_conj));

    else
         G_pz        =  (s-z_1)*(s-z_1_conj)/((s-p_1)*(s-p_1_conj)*(s-p_2)*(s-p_2_conj));
    end
    G_pz_nongain(k)= G_pz;
    [num1,den1]    = tfdata(G_pz, 'v');
    H_S_zeros      = freqs(num1,den1,w_fit);
    gain_pz(k)     = db2mag(0)/abs(H_S_zeros(1)); % Gain to ensure the system will start from zero db
    G_pz           = gain_pz(k)*G_pz; 
    [num1,den1]    = tfdata(G_pz, 'v');
    H_S_pred(:,k)  = freqs(num1,den1,w_fit);
    G_pz_save(k)   = G_pz;
%     H_S_pred  = freqs(num,den,w_fit);
%     H_S_pred  = H_S_pred*db2mag(0)/abs(H_S_pred(1));
%     H_S_zeros = freqs(num1,den1,w_fit);
%     H_S_zeros = H_S_zeros*db2mag(0)/abs(H_S_zeros(1));
    
%     [p_pred,~] = pzmap(frf_pred);
%     scale_nf   = nat_freq_from_elec_shoulder^2 / abs(p_pred(1))^2;

%    sys_pred           = zpk(z{1,1},[p{1,1}; p_pred],g/scale_nf);
%     H_S_pred           = freqresp(sys_pred,freq_range_fit,'Hz');
%     H_S_pred_save(:,k) = H_S_pred(:);
% 
%     figure(122)
%     subplot(2,1,1)
%     c1=semilogx(freq_range,mag2db(abs(H_S(:,k))),'LineWidth',2);
%     hold on
%     c2=semilogx(freq_range_fit,mag2db(abs(H_S_pred(:))),'k--','LineWidth',2);
%     c3=text(3, -40, ['q_1: ', num2str(round(joint_positions_shoulder(k,1),2)), ', q_2: ', num2str(round(joint_positions_shoulder(k,2),2)),...
%         ', q_3: ', num2str(round(joint_positions_shoulder(k,3),2)), ', q_4: ', num2str(round(joint_positions_shoulder(k,4),2)),... 
%         ', q_5: ', num2str(round(joint_positions_shoulder(k,5),2)), ', q_6: ', num2str(round(joint_positions_shoulder(k,6),2))]);
%     ylabel('Mag [dB]')
%     grid on
%     xlim([2 60])
%     title('Shoulder Joint FRFs (Fits & Predictions)')
%     hold off
% 
%     subplot(2,1,2)
%     h1=semilogx(freq_range,unwrap(angle(H_S(:,k)))/pi*180,'LineWidth',2);
%     hold on
%     h2=semilogx(freq_range_fit,unwrap(angle(H_S_pred(:)))/pi*180,'k--','LineWidth',2);
%     xlabel('Frequency [Hz]')
%     ylabel('Phase [deg]')
%     grid on
%     xlim([2 60])
%     hold off
%     pause(0.0001);
% 
%     if k >= 1
%         delete(c3)
%     end

        figure(3)
        subplot(2,2,1)
        semilogx(freq_range,mag2db(abs(H_S(:,k))),'b','LineWidth',2);hold on
        semilogx(w_fit/(2*pi),mag2db(abs(H_S_fit(:,k))),'r--','LineWidth',2);
        semilogx(w_fit/(2*pi),mag2db(abs(H_S_pred(:,k))),'k:','LineWidth',2);hold off
%         semilogx(w_fit/(2*pi),mag2db(abs(H_S_pred)),'k-','LineWidth',2);
%         semilogx(w_fit/(2*pi),mag2db(abs(H_S_zeros)),'r:','LineWidth',2);hold off
%         ylabel('Mag [dB]')
        grid on
        xlim([2 60])
        title(['H FRF - Shoulder FRFs R = ',num2str(R_m_base(k)),' V = ',num2str(V_rad_base(k)*180/pi),''])
        legend('sys ID','Fited','Prediction',"Location","southwest")
        
        subplot(2,2,3)
        semilogx(freq_range,unwrap(angle(H_S(:,k)))/pi*180,'b','LineWidth',2);hold on
        semilogx(w_fit/(2*pi),unwrap(angle(H_S_fit(:,k)))/pi*180,'r--','LineWidth',2);
        semilogx(w_fit/(2*pi),unwrap(angle(H_S_pred(:,k)))/pi*180,'k:','LineWidth',2);hold off
%         semilogx(w_fit/(2*pi),unwrap(angle(H_S_zeros))/pi*180,'r:','LineWidth',2);hold off
        xlabel('Frequency [Hz]')
        ylabel('Phase [deg]');
        grid on
        xlim([2 60]);
        pause(0.01)

        subplot(2,2,2)
        semilogx(freq_range,mag2db(abs(H_S_full(:,k))),'b','LineWidth',2);hold on
%        semilogx(w_fit/(2*pi),mag2db(abs(H_S_fit(:,k))),'r--','LineWidth',2);
        semilogx(freq_range,mag2db(abs(H_S_pred(2:end,k).*H_S_joint(:,k))),'k:','LineWidth',2);hold off
%         semilogx(w_fit/(2*pi),mag2db(abs(H_S_pred)),'k-','LineWidth',2);
%         semilogx(w_fit/(2*pi),mag2db(abs(H_S_zeros)),'r:','LineWidth',2);hold off
%         ylabel('Mag [dB]')
        grid on
        xlim([2 60])
        title(['H_{full} - Shoulder FRFs R = ',num2str(R_m_base(k)),' V = ',num2str(V_rad_base(k)*180/pi),''])
        legend('sys ID','Prediction',"Location","southwest")
        
        subplot(2,2,4)
        semilogx(freq_range,unwrap(angle(H_S_full(:,k)))/pi*180,'b','LineWidth',2);hold on
%        semilogx(w_fit/(2*pi),unwrap(angle(H_S_fit(:,k)))/pi*180,'--','LineWidth',2);
        semilogx(freq_range,unwrap(angle(H_S_pred(2:end,k).*H_S_joint(:,k)))/pi*180,'k:','LineWidth',2);hold off
%         semilogx(w_fit/(2*pi),unwrap(angle(H_S_zeros))/pi*180,'r:','LineWidth',2);hold off
        xlabel('Frequency [Hz]')
        ylabel('Phase [deg]');
        grid on
        xlim([2 60]);
        pause(0.01)

            %%%%%%%%%%% This is used to generate a gif
%     if k==1
%         gif('Predictions.gif','DelayTime',1/5)
%     else
%         gif
%     end
    %%%%%%%%%%%
        
end

%% Rebuilding Elbow FRFs
G_pz_nongain_elbow = tf(length(shoulder_transfer_functions),1); % TF correction to make sure we will start from 0 db
cont               = 0;
H_E_pred           = zeros(size(H_S_fit));
G_pz_nogain        = tf(zeros(length(shoulder_transfer_functions),1)); 
G_pz_save_elbow    = tf(zeros(length(shoulder_transfer_functions),1));
gain_pz_elbow      = zeros(size(gain_pz));

for k = 1:length(elbow_transfer_functions)

    % Define the natural poles and zeros
    s           = tf([1 0],[1]);
    img         = sqrt(-1);

    % Pole/zero model = -zeta*wn+wn*sqrt(1-zeta^2)
    % Complex conjugate zero
    zeta        = z_zeta_complex_conjugate_elbow1(k);
    wn          = abs(z_complex_conjugate_elbow_1(k));
    z_1         = -zeta*wn + wn*sqrt(1-zeta^2)*img;  
    z_1_conj    = -zeta*wn - wn*sqrt(1-zeta^2)*img;
    
    % Complex conjugate pole 1 
    zeta        = zeta_elbow(k);
    wn          = nat_freqs_elbow_high(k);
    p_1         = -zeta*wn + wn*sqrt(1-zeta^2)*img;  
    p_1_conj    = -zeta*wn - wn*sqrt(1-zeta^2)*img;
    
    % Complex conjugate pole 2 
    zeta        = zeta_elbow_max(k);
    wn          = nat_freqs_elbow_max(k);
    p_2         = -zeta*wn + wn*sqrt(1-zeta^2)*img;  
    p_2_conj    = -zeta*wn - wn*sqrt(1-zeta^2)*img;
    
    
    G_pz        =  (s-z_1)*(s-z_1_conj)/((s-p_1)*(s-p_1_conj)*(s-p_2)*(s-p_2_conj));
   
    G_pz_nongain_elbow(k) = G_pz;
    [num1,den1]           = tfdata(G_pz, 'v');
    H_E_zeros             = freqs(num1,den1,w_fit);
    gain_pz_elbow(k)      = db2mag(0)/abs(H_E_zeros(1)); % Gain to ensure the system will start from zero db
    G_pz                  = gain_pz_elbow(k)*G_pz; 
    [num1,den1]           = tfdata(G_pz, 'v');
    H_E_pred(:,k)         = freqs(num1,den1,w_fit);
    G_pz_save_elbow(k)    = G_pz;

    figure(1222)
    subplot(2,2,1)
    c1=semilogx(freq_range,mag2db(abs(H_E(:,k))),'LineWidth',2);
    hold on
    c2=semilogx(freq_range_fit,mag2db(abs(H_E_pred(:,k))),'k--','LineWidth',2);
%     c3=text(3, -40, ['q_1: ', num2str(round(joint_positions_shoulder(k,1),2)), ', q_2: ', num2str(round(joint_positions_shoulder(k,2),2)),...
%         ', q_3: ', num2str(round(joint_positions_shoulder(k,3),2)), ', q_4: ', num2str(round(joint_positions_shoulder(k,4),2)),... 
%         ', q_5: ', num2str(round(joint_positions_shoulder(k,5),2)), ', q_6: ', num2str(round(joint_positions_shoulder(k,6),2))]);
    ylabel('Mag [dB]')
    grid on
    xlim([2 60])
    title('Elbow Joint FRFs - FRF H')
    hold off

    subplot(2,2,3)
    h1=semilogx(freq_range,unwrap(angle(H_E(:,k)))/pi*180,'LineWidth',2);
    hold on
    h2=semilogx(freq_range_fit,unwrap(angle(H_E_pred(:,k)))/pi*180,'k--','LineWidth',2);
    xlabel('Frequency [Hz]')
    ylabel('Phase [deg]')
    grid on
    xlim([2 60])
    hold off
    pause(0.1);

    subplot(2,2,2)
    c1=semilogx(freq_range,mag2db(abs(H_E_full(:,k))),'LineWidth',2);
    hold on
    c2=semilogx(freq_range,mag2db(abs(H_E_pred(2:end,k).*H_E_joint(:,k))),'k--','LineWidth',2);
%     c3=text(3, -40, ['q_1: ', num2str(round(joint_positions_shoulder(k,1),2)), ', q_2: ', num2str(round(joint_positions_shoulder(k,2),2)),...
%         ', q_3: ', num2str(round(joint_positions_shoulder(k,3),2)), ', q_4: ', num2str(round(joint_positions_shoulder(k,4),2)),... 
%         ', q_5: ', num2str(round(joint_positions_shoulder(k,5),2)), ', q_6: ', num2str(round(joint_positions_shoulder(k,6),2))]);
    ylabel('Mag [dB]')
    grid on
    xlim([2 60])
    title('Elbow Joint FRFs- H_full')
    hold off

    subplot(2,2,4)
    h1=semilogx(freq_range,unwrap(angle(H_E_full(:,k)))/pi*180,'LineWidth',2);
    hold on
    h2=semilogx(freq_range,unwrap(angle(H_E_pred(2:end,k).*H_E_joint(:,k)))/pi*180,'k--','LineWidth',2);
    xlabel('Frequency [Hz]')
    ylabel('Phase [deg]')
    grid on
    xlim([2 60])
    hold off
    pause(0.1);

    
% 
%     if k >= 1
%         delete(c3)
%     end


%         figure(3)
%         subplot(2,1,1)
%         semilogx(freq_range,mag2db(abs(H_S(:,k))),'b','LineWidth',2);hold on
%         semilogx(w_fit/(2*pi),mag2db(abs(H_S_fit(:,k))),'r--','LineWidth',2);
%         semilogx(w_fit/(2*pi),mag2db(abs(H_S_pred(:,k))),'k:','LineWidth',2);hold off
% %         semilogx(w_fit/(2*pi),mag2db(abs(H_S_pred)),'k-','LineWidth',2);
% %         semilogx(w_fit/(2*pi),mag2db(abs(H_S_zeros)),'r:','LineWidth',2);hold off
% %         ylabel('Mag [dB]')
%         grid on
%         xlim([2 60])
%         title(['Shoulder FRFs R = ',num2str(R_m(k)),' V = ',num2str(V_rad(k)*180/pi),''])
%         legend('sys ID','Fited','Prediction',"Location","southwest")
%         
%         subplot(2,1,2)
%         semilogx(freq_range,unwrap(angle(H_S(:,k)))/pi*180,'LineWidth',2);hold on
%         semilogx(w_fit/(2*pi),unwrap(angle(H_S_fit(:,k)))/pi*180,'--','LineWidth',2);
%         semilogx(w_fit/(2*pi),unwrap(angle(H_S_pred(:,k)))/pi*180,'k-','LineWidth',2);hold off
% %         semilogx(w_fit/(2*pi),unwrap(angle(H_S_zeros))/pi*180,'r:','LineWidth',2);hold off
%         xlabel('Frequency [Hz]')
%         ylabel('Phase [deg]');
%         grid on
%         xlim([2 60]);
%         pause(0.01)
            %%%%%%%%%%% This is used to generate a gif
%     if k==1
%         gif('Predictions.gif','DelayTime',1/5)
%     else
%         gif
%     end
    %%%%%%%%%%%
        
end



% [B,r] = sort(R_m_shoulder,'ascend'); %% Finding the index to put R in a ascending order
% [B,v] = sort(V_rad_shoulder,'ascend'); %% Finding the index to put V in a ascending order
% 
% 
% for k=1:length(V_rad_shoulder)
% 
%     index = v(k);
%     
%          figure(2)
%          subplot(2,1,1)
%          semilogx(freq_range,mag2db(abs(H_E(:,index))),'b','LineWidth',2);hold on
%          semilogx(w_fit/(2*pi),mag2db(abs(H_E_fit(:,index))),'r--','LineWidth',2);
%          semilogx(w_fit/(2*pi),mag2db(abs(H_E_pred(:,index))),'k:','LineWidth',2);hold off
%          
%          yline(-3,'b-.','-3 dB','LineWidth',2,'LabelHorizontalAlignment','left',...
%              'LabelVerticalAlignment','bottom');hold off  % plots a line at 3 dB;hold off
%          xline(45,'r-.','45 Hz','LineWidth',2,'LabelVerticalAlignment','bottom'...
%              ,'LabelOrientation','horizontal','LabelHorizontalAlignment','left');  % plots a line at 3 dB;hold off
%          
%          ylabel('Mag [dB]')
%          grid on
%          xlim([2 60])
%          title(['Elbow FRFs R = ',num2str(R_m_shoulder(index)),' V = ',num2str(V_rad_shoulder(index)*180/pi),'\color{darkGreen}{ FIXED V} \color{blue}{--} \color{red}{DECREASING R}'])
%          legend('H_E','H_E_{fit}','H_E_{prediction}','Location','southwest')
%             
%          subplot(2,1,2)
%          semilogx(freq_range,unwrap(angle(H_E(:,index)))/pi*180,'b','LineWidth',2);hold on
%          semilogx(w_fit/(2*pi),unwrap(angle(H_E_fit(:,index)))/pi*180,'r--','LineWidth',2);
%          semilogx(w_fit/(2*pi),unwrap(angle(H_E_pred(:,index)))/pi*180,'k:','LineWidth',2);hold off
%          xlabel('Frequency [Hz]')
%          ylabel('Phase [deg]');
%          grid on
%          xlim([2 60]);
% 
%          subplot(2,1,1)
%          semilogx(freq_range,mag2db(abs(H_E(:,index))),'b','LineWidth',2);hold on
%          semilogx(w_fit/(2*pi),mag2db(abs(H_E_fit(:,index))),'r--','LineWidth',2);
%          semilogx(w_fit/(2*pi),mag2db(abs(H_E_pred(:,index))),'k:','LineWidth',2);hold off
%          
%          yline(-3,'b-.','-3 dB','LineWidth',2,'LabelHorizontalAlignment','left',...
%              'LabelVerticalAlignment','bottom');hold off  % plots a line at 3 dB;hold off
%          xline(45,'r-.','45 Hz','LineWidth',2,'LabelVerticalAlignment','bottom'...
%              ,'LabelOrientation','horizontal','LabelHorizontalAlignment','left');  % plots a line at 3 dB;hold off
%          
%          ylabel('Mag [dB]')
%          grid on
%          xlim([2 60])
%          title(['Elbow FRFs R = ',num2str(R_m_shoulder(index)),' V = ',num2str(V_rad_shoulder(index)*180/pi),'\color{darkGreen}{ FIXED V} \color{blue}{--} \color{red}{DECREASING R}'])
%          legend('H_E','H_E_{fit}','H_E_{prediction}','Location','southwest')
%             
%          subplot(2,1,2)
%          semilogx(freq_range,unwrap(angle(H_E(:,index)))/pi*180,'b','LineWidth',2);hold on
%          semilogx(w_fit/(2*pi),unwrap(angle(H_E_fit(:,index)))/pi*180,'r--','LineWidth',2);
%          semilogx(w_fit/(2*pi),unwrap(angle(H_E_pred(:,index)))/pi*180,'k:','LineWidth',2);hold off
%          xlabel('Frequency [Hz]')
%          ylabel('Phase [deg]');
%          grid on
%          xlim([2 60]);
%       
% %     if k==1
% %         gif('Elbow_FRFs_predictions_fixed_V.gif','DelayTime',1/5)
% %     else
% %         gif
% %     end
% 
% end

% figure(250)
% subplot(2,1,1)
% semilogx(freq_range_fit,mag2db(abs(H_S_pred_save)),'LineWidth',2)
% ylabel('Mag [dB]')
% grid on
% xlim([2 60])
% title('Shoulder Joint FRFs (Least Squares Prediction)')
% 
% subplot(2,1,2)
% semilogx(freq_range_fit,unwrap(angle(H_S_pred_save))/pi*180,'LineWidth',2)
% xlabel('Frequency [Hz]')
% ylabel('Phase [deg]')
% grid on
% xlim([2 60])


% save('Elbow_TF','G_pz_save_elbow')
% save('Elbow_FRF','H_E')


%save('Shoulder_TF','G_pz_nongain','G_pz_save')
%save('shoulder_FRF_complex_conj_zero','z_zeta_complex_conjugate_shoulder1','z_complex_conjugate_1')

%% Save models
[z_elec_shoulder,p_elec_shoulder,g_elec_shoulder] = zpkdata(sys_elec_shoulder);
z_elec_shoulder = z_elec_shoulder{1};
p_elec_shoulder = p_elec_shoulder{1};
% save(['shoulder_damping_stiffness_params_',date_str], 'k0_shoulder', 'k1_shoulder',...
%     'c0_shoulder','c1_shoulder', 'c2_shoulder', 'c_tau_shoulder', 'H_S',...
%     'freq_range_fit','freq_range', 'H_S_fit',...
%     'z_elec_shoulder','p_elec_shoulder','g_elec_shoulder','nat_freq_from_elec_shoulder','J1_shoulder','joint_positions_shoulder')