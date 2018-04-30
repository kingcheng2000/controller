%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% heli.m
%% Matlab script to be run before Simulink files
%% ACS336 / ACS6336 / ACS6110
%% Last revised: 23.02.2015
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clc;
close all;      % close all figures
clear all;      % clear workspace variables

%% Define Discrete Time MyDAQ Dynamics
T           = 0.015;            % Sample period (s)
ADC_Vres    = 20/((2^16)-1);    % ADC voltage resolution (V/bit)
Encoder_res = 2*pi/500;         % Encoder resolution (rad/wheel count)
DAC_Vres    = 20/((2^16)-1);    % DAC voltage resolution (V/bit)
DAC_lim_u   = 10;               % DAC upper saturation limit (V)
DAC_lim_l   = 0;                % DAC enforced lower saturation limit (V)
%% Define Continuous Time Helicopter Dynamics
g   = 9.81;     % Gravitational acceleration (ms^-2) 
% Rigid body parameters
% Masses and lengths
m1  = 0.0505;   % mass of fan assembly (kg)
m2  = 0.100;    % mass of counterweight (kg)
l1  = 0.110;    % distance from helicopter arm to elevation axis (m);
l2  = 0.070;    % distance from fan centres to pitch axis (m);
l3  = 0.108;    % distance from counterweight to elevation axis (m);
% Inertias
Je  = 2*m1*(l1^2)+m2*(l3^2);    % Inertia about elevation axis (kg*m^2);
Jt  = Je;                       % Travel axis inertia
Jp  = 2*m1*(l2^2);              % Pitch axis inertia
% Constraints
p_lim_u     = 80*pi/180;    % Upper pitch axis limit (rad)
p_lim_l     = -80*pi/180;   % Lower pitch axis limit (rad)
e_lim_u     = 50*pi/180;    % Upper elevation axis limit (rad)
e_lim_l     = -50*pi/180;   % Lower elevation axis limit (rad)

% %% Ex 1: DETERMINE PITCH AXIS SPRING AND DAMPING COEFFICIENTS %%%%%%%%%%%%%
% % Pitch axis spring and damping constants
 k_s = 0.0008;           % Spring constant (kg*m^2*s^-2)
 k_d = 0.0005;           % Viscous damping (kg*m^2*s^-1)
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 
% %% Ex 2: DETERMINE POWER AMPLIFIER GAIN AND SATURATION LIMITS %%%%%%%%%%%%%
% % Power amplifier
k_a         = 1.2;   % Power amplifier voltage gain
amp_sat_u   = 12;   % Power amplifier upper saturation limit (V)
amp_sat_l   = 0;   % Power amplifier lower saturation limit (V)
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 
% %% Ex 3: CONSTRUCT FAN MODEL %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% % Fan voltage - thrust steady state behaviour

%Load the file from the helicopter that runed for 10 seconds with Step
%input of 10 volts We obtained raw data from the gauge. This in order to
%symulate the Fan dynamics in a simplistic way.
FanDynData = load('FanDynData.txt');
TimeArray = FanDynData(1,:);
Excitation = FanDynData(2,:);
Fan_Thrust = FanDynData(3,:);
Filtered_Data = FanDynData(4,:);

figure(1)
plot(TimeArray(101:1001)-1,Fan_Thrust(101:1001))
hold on; grid on;

%Data obtained on a experiment by inputing voltages from 0V to 12V in
%intervals of 0.5V and ploting the thrust output obtained from a gauge. The
%values in this files are in volts and Newtons.
FanSteadyState = load('FanSteadyState.txt');
V_ab   = FanSteadyState(1,:);          % Fan voltage input (V)
Fss_ab = FanSteadyState(2,:); % Steady-state fan thrust output(N) Values were obtined from the file "Fan characterization - Dynamic" 

% % Fan voltage - thrust transient model.
tau = 1/1.7;            % 1st order time constant
%
% %% Ex 4: DETERMINE EQUILIBRIUM CONTROL SIGNAL %%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% % Constant control input required to achieve hover
%U_e = 7.5;           % Voltage output from myDAQ
U_e = 3.7;

% %% Ex 5: DEFINE LTI STATE-SPACE CONTROL MODEL %%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% %  Approximate the fan voltage/thrust relationship by an affine           %
% %  function of the form F_ab = alpha*V_ab+beta. Define alpha and beta.    %
alpha = 0.0031;
beta =  -.0060;
F_ab = (alpha.*V_ab.*k_a)+beta;
Fss = (alpha.*Excitation(101:1001).*k_a+beta);

Fs = Fss.*(1-exp(-(TimeArray(101:1001)-1)/tau));

plot(TimeArray(101:1001)-1,Filtered_Data(101:1001))
hold on;
plot(TimeArray(101:1001)-1,Fs)
xlabel('Time (s)')
ylabel('Force (N)')
legend('Force (N)','Filtered Force (N)')

figure(2)
plot(V_ab,Fss_ab, 'kx') % plot raw thrust data
grid on; hold on;
plot(V_ab,Fss_ab, 'r-')
hold on;

plot(V_ab,F_ab,'b-'); % plot linear approximation
legend('Fan Steady State Point','Fan Steady State Function','Fan Linear Aproximation')
xlabel('Fan Voltage (V)')
ylabel('Fan Thrust (N)')
% %  State vector x:=[elev; pitch; trav; elev_dot; pitch_dot; trav_dot]     %
% %  Note; these states model the dynamics of small perturbations around    %
% %  the state of steady, level hover.                                      %
% %  Define the control model given by x_dot = Ax + Bu, y = Cx + Du         %
x1= -0.0015/(4.95*10^-4);
x2= -0.0005/(4.95*10^-4);
x3= (-2*0.11*(0.0028*1.2*5.5-0.0057))/0.0024;
b1= (.11*0.0028*1.2)/0.0024;
b2= (0.07*0.0028*1.2)/(4.95*10^-4);

A =  [ 0, 0,  0, 1, 0, 0;
       0, 0,  0, 0, 1, 0;
       0, 0,  0, 0, 0, 1;
       0, 0,  0, 0, 0, 0;
       0, x1, 0, 0, x2, 0;
       0, x3, 0, 0, 0, 0];

B =  [0,  0;
      0,  0;
      0,  0;
      b1, b1;
      b2, -b2;
      0,  0];

C =  [1, 0, 0, 0, 0, 0;
      0, 1, 0, 0, 0, 0;
      0, 0, 1, 0, 0, 0];
 
D =  [0, 0
      0, 0
      0, 0];
  
% %% Ex 6: Discrete Time Full state feedback control %%%%%%%%%%%%%%%%%%%%%%%%
% % State feedback control design (integral control via state augmentation)

%%x=[elev; pitch; trav; elev_dot; pitch_dot; trav_dot; int_elev; int_trav];

% % Define augmented system matrices
 Cr      = [1 0 0 0 0 0
            0 0 1 0 0 0];                    % Elevation and travel are controlled outputs
 r       = 2;                                % number of reference inputs
 n       = size(A,2);                        % number of states
 q       = size(Cr,1);                       % number of controlled outputs
 Dr      = zeros(q,2);
 Aaug    = [A zeros(n,r); -Cr zeros(q,r)];
 Baug    = [B; -Dr];
 Caug = [Cr zeros(q,r)];
 Daug = D;
 
% % Define LQR weighting matrices

 Qx = eye(8);    % State penalty - Initialized the Matrix
 Qx(1,1) = 1;   % Penalization for elevation      
 Qx(2,2) = 12;   % Penalization for pitch
 Qx(3,3) = 1;    % Penalization for travel
 Qx(4,4) = 90;    % Penalization for elevation velocity
 Qx(5,5)= 300;   % Penalization for pitch velocity
 QX(6,6)= 200;   % Penalization for travel velocity
 Qu = eye(2);    % Control penalty - Created as Identity Matrix
  
% % Discrete-Time LQR synthesis
 Kdtaug  = lqrd(Aaug,Baug,Qx,Qu,T);      % DT state-feedback controller
 Kdt     = Kdtaug(:,1:n); 
 Kidt = -Kdtaug(:,n+1:end);
%  Discrete-Time Kalman Filter Design
 sysdt = c2d(ss(A,B,C,D),T,'zoh');     % Generate discrete-time system
 Adt   = sysdt.a; %Extract the discrete A matrix
 Bdt = sysdt.b; 
 Cdt = sysdt.c; 
 Ddt = sysdt.d;
% %  Kalman filter design; x_dot = A*x + B*u + G*w, y = C*x + D*u + H*w + v
 Gdt = 1e-1*eye(n);
 Hdt = zeros(size(C,1),size(Gdt,2)); % No process noise on measurements
 Rw  = eye(6);   % Process noise covariance matrix
 Rw(1,1)= Qx(1,1);
 Rw(2,2)= Qx(2,2);
 Rw(3,3)= Qx(3,3);
 Rw(4,4)= Qx(4,4);
 Rw(5,5)= Qx(5,5);
 Rw(6,6)= QX(6,6); 
 Rv  = eye(3);   % Measurement noise covariance matrix
 sys4kf  = ss(Adt,[Bdt Gdt],Cdt,[Ddt Hdt],T);
 [kdfilt Ldt] = kalman(sys4kf,Rw,Rv);     % Kalman filter synthesis
% 
%% Output Files %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% % Uncomment these lines when ready to implement your feedback controller  %
k_amp       = k_a;
Vfan        = V_ab;
Thrust      = Fss_ab;
% 
KF_Initial  = [e_lim_l 0 0 0 0 0];
Kdi_Initial = [0 0];
%

csvwrite('aMatrix.txt',kdfilt.a)
csvwrite('bMatrix.txt',kdfilt.b)
csvwrite('cMatrix.txt',kdfilt.c(size(C,1)+1:end,:))
csvwrite('dMatrix.txt',kdfilt.d(size(C,1)+1:end,:))
csvwrite('crMatrix.txt',Cr)
csvwrite('KdMatrix.txt',Kdt)
csvwrite('KdiMatrix.txt',Kidt)
csvwrite('FanChar.txt',[Vfan,Thrust])
csvwrite('ModelParameters.txt',[T,U_e,l1,l2,l3,Jp,Jt,Je,m1,m2,g,...
k_amp, amp_sat_u, amp_sat_l, DAC_lim_u, DAC_lim_l,...
e_lim_l, e_lim_u, p_lim_l, p_lim_u])
csvwrite('KF_Initial.txt',KF_Initial)
csvwrite('Kdi_Initial.txt',Kdi_Initial)