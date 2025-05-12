%% Curve fitting 2.0
% In this code, I will generate the neural network models based on the data
% extracted from the
% Fitting_base_and_shoulder_FRF_With_different_TF_order_V9.

clear all
close all
clc 

% load (['D:\OneDrive - Umich\PhD\System ID 5 kg\Data\09-Aug-2023\R_V_NatFreq_Damping_09_Aug_base.mat'])
% load (['D:\OneDrive - Umich\PhD\System ID 5 kg\Data\09-Aug-2023\R_V_NatFreq_Damping_09_Aug_elbow.mat'])
% load (['D:\OneDrive - Umich\PhD\System ID 5 kg\Data\09-Aug-2023\R_V_NatFreq_Damping_09_Aug_shoulder.mat'])
load R_V_NatFreq_Damping_14_jun_shoulder.mat

% % Base
% nat_freqs_base_high ok
% zeta_base ok
%
% % Shoulder
% nat_freqs_shoulder_high ok
% zeta_shoulder ok
% 
% % elbow
% nat_freqs_elbow_high ok
% zeta_elbow ok
% 
% nat_freqs_elbow_max ok
% zeta_elbow_max ok
% 

cont = 1;

if cont == 1
    Z = '\omega_n [Hz]';
    X = 'V [degrees]';
    Y = 'R [mm]';
else
    Z = '\zeta [-]';
    X = 'V [degrees]';
    Y = 'R [mm]';
end

name    = 'Shoulder Joint First pole';
targets = abs(nat_freqs_shoulder_high);

figure
if cont ==1
    plot3(V_rad_shoulder*(180/pi),R_m_shoulder*1000,targets/(2*pi), '*','LineWidth',2);hold on
else
    plot3(V_rad_shoulder*(180/pi),R_m_shoulder*1000,targets, '*','LineWidth',2);hold on
end
%plot3(V_rad_shoulder*180/pi, R_m_shoulder*1000,nat_freqs_shoulder_high/(2*pi))
xlabel('Angle from horizontal, V [deg]')
ylabel('Radius from base, R [mm]')
zlabel('Natural frequency [Hz]')
%zlim([10 40])
grid on

%targets = nat_freqs_shoulder_high/(2*pi);
%targets = zeta_shoulder_max;
%targets = z_complex_conjugate_1;
%targets = abs(z_complex_conjugate_1)'/(2*pi);
inputs  = [V_rad_shoulder * 180 / pi, R_m_shoulder * 1000];


% Set the threshold values for outlier detection
% damping ratio of zers
% lowerThreshold = 0.1;  
% upperThreshold = 0.36;

% This part of the code can be used to remove outliers from the dataset. 
% % Nat freq of zeros 
% lowerThreshold = 10*2*pi;  % [Hz]
% upperThreshold = 35*2*pi;  % [Hz]
% 
% % Identify outliers based on the threshold values
% outlierIdx = targets < lowerThreshold | targets > upperThreshold;
% % outlierIdx = abs(z_complex_conjugate_elbow_1) < lowerThreshold | abs(z_complex_conjugate_elbow_1) > upperThreshold;
% % % 
% % % % Remove outliers from the dataset
% inputs  = inputs(~outlierIdx, :);
% targets = targets(~outlierIdx);


% First version: No hidden layers
net1 = fitnet(10); % Empty hidden layer sizes

% Train the first neural network
net1 = train(net1, inputs', targets');

% Generate surface predictions for the first version
x_test            = linspace(min(inputs(:, 1)), max(inputs(:, 1)), 100)';
y_test            = linspace(min(inputs(:, 2)), max(inputs(:, 2)), 100)';
[X_test, Y_test]  = meshgrid(x_test, y_test);
inputs_test       = [X_test(:), Y_test(:)]';
predictions_test1 = net1(inputs_test);
Z_predicted_test1 = reshape(predictions_test1, size(X_test));

% Plot original data and fitted surfaces for the first version
figure;
if cont == 1
    scatter3(inputs(:, 1), inputs(:, 2), targets/(2*pi), 'r*', 'LineWidth', 2);hold on;
    surf(X_test, Y_test, Z_predicted_test1/(2*pi));
else
    scatter3(inputs(:, 1), inputs(:, 2), targets, 'r*', 'LineWidth', 2);hold on;
    surf(X_test, Y_test, Z_predicted_test1);
end

hold off;
title(['NN model - ',replace(name,'_',' '),'']);
xlabel(X);
ylabel(Y);
zlabel(Z);

% Adjust plot settings for better visualization
set(gcf, 'color', 'w');
colormap('jet');
shading interp;
axis tight;


% Generate predictions for the first version
predictions1 = net1(inputs');
mse1         = mean((predictions1 - targets').^2);
variance1    = var(predictions1);

% Print the MSE and variance for the first version
fprintf('First version:\n');
fprintf('MSE: %.4f\n', mse1);
fprintf('Variance: %.4f\n\n', variance1);

% Assuming you have already trained and obtained the neural network model 'net'
% Define new input values for prediction
newInput = [42.5, 475]; % Replace x(1) and y(1) with your desired input values

% Perform prediction using the trained neural network
predictedOutput = net1(newInput');

% Display the predicted output
disp(['Predicted Output: ' num2str(predictedOutput)]);

%wn_base_1   = net1; ok
%zeta_base_1  = net1; ok

% % Shoulder
% wn_shoulder_1     = net1; ok
% zeta_shoulder_1   = net1; ok 
% 
% wn_shoulder_2     = net1; N/A
% zeta_shoulder_2   = net1; N/A
% 
% z_wn_shoulder_1   = net1; N/A
% z_zeta_shoulder_1 = net1; N/A
% 
% % elbow 
% wn_elbow_1        = net1; 
% zeta_elbow_1      = net1;
% 
%wn_elbow_2        = net1;
%zeta_elbow_2      = net1;
%  
% z_wn_elbow_1      = net1; N/A
% z_zeta_elbow_1    = net1; N/A

%save(['D:\OneDrive - Umich\PhD\System ID 5 kg\Scripts\NN_models_IP\',name,''],"zeta_elbow_2");

