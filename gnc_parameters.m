clc;
close all;
clear all;
format short 

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%% Guidance, Navigation and Control System for           %%%%
%%%           using GNSS and obstacle avoidance             %%%          
%%% Author : Nathan FAGNOU, based on the work of Vinayak - D %%
%%% His github : https://github.com/Vinayak-D/GNCAirstrike %%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Definition of the parameters with Reichert's M..... Benchmark
A = [ -1.064, 1 ; 290.26, 0] ; % ( System's dynamic matrice )
B = [-0.25; -331.40] ; % (Control surfaces inputs ) 
C = [-123.34, 0 ; 0, 1] ; % (Acceleration and velocity matrice )
D = [-13.51; 0] ; % (Direct transmission vector)

states = {'AoA', 'q'} ; % Definition of states (Angle of attack and pitch rate (radius))
inputs = {'\Delta_c'} ; % Definition of inputs (Angle deflexion (radius))
outputs = {'a_z', 'q'} ; % Definition of the outputs (normal acceleration 

sys = ss(A,B,C,D, 'statename',states, 'inputname', inputs,'outputname', outputs ); % Definition of the dynamic system

%% Transfer function 

TFs = tf(sys) ; % Computation of every TFs (1 input and 2 outputs so multiple TFs ; a_z/delta_c and q/delta_c)
TF = TFs(2,1) ; % q/delta_c (More useful for to stabilize the missile than Tfs(1,1)
disp(pole(TF)) ; % Stability analysis ( Computation of the eigeen values of the denominator of the Tf)

%% LQR weight matrices 
Q = [0.1, 0 ; 0, 0.1] ; 
R = 0.5 ;

%% LQR gain 
[K, S, e] = lqr(A,B,Q,R) ;
fprintf('eigenvalues of A-BK\n') ; 
disp(eig(A-B*K)) ; 
fprintf('Feedback gain K') ;
disp(K) ;

%% Closed loop system
Acl = A-B*K ; 
Bcl = B ;

syscl = ss(Acl,Bcl,C,D, 'statename',states, 'inputname', inputs,'outputname', outputs ); % Definition of the dynamic closed loop system

%% LQG Kalman filter design 
G = eye(2) ; % pertubations inputs matrice (How wind affects the m.... dyanmic's)
H = 0*eye(2) ; % pertubations's transmission matrice (how wind affects the measurement)

%% Kalman Q,R noise matrices 
Qbar = diag(0.00015*ones(1,2)) ; 
Rbar = diag(0.55*ones(1,2)); 

%% Define noisy system 
sys_n = ss(A,[B, G], C, [D, H]) ; 
[kest, L, P] = kalman(sys_n, Qbar, Rbar, 0);

%% Kalman gain observer closed loop
Aob = A-L*C ; 

%% Display observer eigenvalues
fprintf('Obersver eigenvalues\n') ; 
disp(eig(Aob)); % Display the eigenvalues of the observer

%% Noise time constants (you choose)
dT1 = 0.75;
dT2 = 0.25;

%% M... parameters 

R = 6371e3 ; % earth radius 
Vel = 3021.08 ; % (m/s) Missile
%Vel = 290; 
m2f = 3.2811; % meters to feet 

%% target location 
Lat_target = 34.6624 ; % Lake 
Lon_target = -118.7629 ; % Lake
%Lat_target = 34.6374 ; % Airforce plant 42
%Lon_target = -118.0834 ;
Elev_target = 800; % m  - MSL (height)

%% Initial location
Lat_init = 34.2329; 
Lon_init = -119.4573; 
Elev_init = 10000; % m p - MSL

%% Obstacle location 
%Lat_obs = 34.61916 ; small mountain 
%Lon_obs = -118.8429 ; small mountain
Lat_obs = 34.608485 ;
Lon_obs = -118.867712 ;

d2r = pi/180 ; % degrees to radians

%% Convert to radians 
l1 = Lat_init*d2r ; 
u1 = Lon_init*d2r ;
l2 = Lat_target*d2r; 
u2 = Lon_target*d2r;

dl = l2 - l1 ;
du = u2 - u1 ; 

%% havershine formula 

a = sin(dl/2)^2 + cos(l1)*cos(l2)*sin(du/2)^2 ;

c = 2*atan2(sqrt(a),sqrt(1-a)) ; 

d = R*c ; % Horizontal distance (in m)

%% initial range (pythagorem theorem ) 
r = sqrt(d^2 +(Elev_target - Elev_init)^2) ;

%% initial azimuth 
yaw_init = azimuth(Lat_init,Lon_init,Lat_target,Lon_target) ; 
yaw = yaw_init*d2r ; 

%% initial flight path angle 
dh = abs(Elev_target-Elev_init) ; 
FPA_init = atan(dh/d) ; % rad






