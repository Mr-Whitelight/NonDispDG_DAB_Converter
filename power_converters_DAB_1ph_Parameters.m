%% Parameters file for power_converters_DAB_1ph.slx
%
clear all;
Fsw= 10e3;         % PWM switching frequency (Hz)
DT= 150e-9;         % Dead time (s)
Scope_Decimation=1; % Scope Decimation
Ts=1e-7;            % Control system sample time (s)
%
Pnom= 1000;          % Nominal power (W)
Vbat= 24;        % DC source 1 nominal voltage (V)

R_DCsrc1=1e-7;      % DC source 1 internal resistance (Ohm)
%
% Switches
Ron_FET= 20e-3;     % FET on-state resistance (Ohm)
Rs_FET= 1e6;        % Snubber resistance Rs (Ohm) : 
Cs_FET= inf;        % Snubber capacitance Cs (F) : 
Ron_Diode= 10e-3;   % Body diode resistance (Ohm) 
Vf_Diode= 4.2;      % Body diode forward voltage drop (V)
%
% Coupling inductor
L_Inductor= 1000e-6;  % Inductance (H)
R_Inductor= 16e-3;  % Resistance (Ohm)
%
% Transformer:
Turn_ratio=15;    % DC Bus nominal voltage is 120V;       
Rprim_Tr=10e-3;    % Primary resistance (Ohm)
Lprim_Tr= 1e-7;   % Primary leakage inductance (H)
Rsec_Tr= 10e-3;     % Secondary resistance (Ohm
Lsec_Tr= 1e-7;     % Secondary leakage inductance (H)
Rm_Tr= 1e6;       % Magnetization resistance (Ohm)
Lm_Tr= 100e-3;     % Magnetization inductance (H)
%
% Filters
% High Voltage Capacitor:
C_HV= 100e-6;      % Capacitance (F)
RC_HV= 1e-3;       % Capacitor ESR (Ohm)
% Low Voltage Capacitor:
C_LV= 2000e-6;     % Capacitance (F)
RC_LV= 1e-3;       % Capacitor ESR (Ohm)