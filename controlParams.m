%% Controller related Parameters
% Controls
par.freq = 500; % control frequency

%
%   (1)<-- b -->(2)
%      \       / ^
%       \     /  |
%       /     \  | l
%      /       \ v
%   (4)         (3)

par.fail_id = [0];      % index of the failured propeller
par.DRF_enable = 0;     % failure of two diagonal rotors?
par.fail_time = 0.0;    % moment failiure occurs

% drone parameters
par.b = 0.1150;     % [m]
par.l = 0.0875;
par.L=0.25*(par.b^2+par.l^2);
par.Ix = 0.0014;    % [kg m^2]
par.Iy = 0.0013;
par.Iz = 0.0025;
par.mass = 0.375;   % [kg]
par.g = 9.81;

par.k0 = 1.9e-6;    % propeller thrust coefficient
par.t0 = 1.9e-8;    % torque coefficient
par.w_max = 7200;   % max / min propeller rotation rates, [rad/s]
par.w_min = 0;

%% INDI reduced att control
par.chi = 105;          % output scheduling parameter, [deg].
par.pos_z_p_gain = 5;%5;   % altitude control pd gains
par.pos_z_d_gain = 3;%3;
par.axis_tilt = 0.0;    % primary axis tilting param, 0 ~ 0.2,  
                        % must be 0 for double rotor failure cases

par.att_p_gain = 200;%200;   % attitude control pd gains 
par.att_d_gain = 30;%30;
par.t_indi = 0.02;      % low-pass filter time constant, [s]

% Yaw control
par.YRC_Kp_r = 5.0;
par.YRC_Kp_psi = 5.0;

% position control
par.position_maxAngle = 30/57.3;    % maximum thrust tilt angle [rad]  
par.position_Kp_pos = [1.5, 1.5, 1.5];  % position control gains
par.position_maxVel = 10;           % maximum velocity
par.position_intLim = 5.0; 
par.position_Ki_vel = [1.0, 1.0, 1.0];  % velocity gains
par.position_Kp_vel = [2.0, 2.0, 2.0];

% Adaptive INDI

% par.mu1=0.98*1e-3*diag([1,1,1,1]);
% par.mu2=0.8*1e-3*diag([1,1,1]);
par.mu1=0.001*diag([1,1,1,1]);
par.mu2=0.001*diag([1,1,1]);
% par.mu1=diag([0,0,0,0]);
% par.mu2=diag([0,0,0]);
Iv = diag([par.Ix,par.Iy,par.Iz]);
par.G1=1e-6*Iv\[-par.b*par.k0 par.b*par.k0 par.b*par.k0 -par.b*par.k0;par.l*par.k0 par.l*par.k0 -par.l*par.k0 -par.l*par.k0;par.t0 -par.t0 par.t0 -par.t0];
par.G2=par.freq*Iv\[0 0 0 0;0 0 0 0;1 -1 1 -1];
%par.G2=1.9*1e-8*[0 0 0 0;0 0 0 0;1 -1 1 -1];
par.zero13 =[0,0,0];
par.G1_0=par.G1;
par.G2_0=par.G2;
par.I=0.1;

% Altitude control

par.w_offset=0;
par.alt_gain=1000; % 1000
par.mode=0; % 0 : using ddz_ctrl_input | 1 : using position
