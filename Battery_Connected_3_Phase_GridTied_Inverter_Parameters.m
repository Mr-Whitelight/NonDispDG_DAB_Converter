%% Parameters for 3-Phase Inverter
clear all
f0 = 50; % System frequency
w0 = 2*pi*f0;
% External Grid Thevenin Impedance
Xgrid = 2.5;    % ohm 
Lgrid = Xgrid/w0; % H 
Rgrid = 0.7;    % ohm 

% Statcom Transformer DYg
Stran    = 1000;  % Rated Power
Vmv      = 380;  % Rated Voltage (MV side)
Vlv      = 200;     % Rated Voltage (LV side)
Rtran_pu = 0.003;   % Total winding resistance
Xtran_pu = 0.6;     % Total leakage inductance
Ntran    = Vmv/Vlv; % Turn ratio

Ltran = (Xtran_pu * (Vlv*Vlv/Stran))/w0;
Rtran = Rtran_pu * (Vlv*Vlv/Stran);

Lgrid_lv = Lgrid * (1/Ntran) * (1/Ntran);  % H referred to LV 
Rgrid_lv = Rgrid * (1/Ntran) * (1/Ntran);  % ohm referred to LV


% Converer parameters
Pinv = Stran;             % Inverter power : 1000 W 
Einv_ltl = Vlv;           % Grid line voltage
Vdc = 400;               %DC link voltage : 400V
fsw = 10000;              %Switching frequency : 10 kHz
fsample = 50000;          %Controller Sampling frequency : 50kHz
wsw = 2*pi*fsw;

%%%%%%%%%%% Grid Integration & Control
% Base values
Sb = Pinv;
Vb = Einv_ltl;
Ib = Sb/Vb/sqrt(3);
Zb = (Vb^2)/Sb;
Cb = 1/(w0*Zb);
Lb = Zb/w0;

Vb_peak = Vb*sqrt(2/3);
Ib_peak = Ib*sqrt(2);

% LCL Filter Parameters  (due to transformer, no grid side inductance)
delta_Ilmax = 0.1*((Sb*sqrt(2/3))/Vb);
Li    = Vdc/(16*fsw*delta_Ilmax); %Inverter side inductance
Li_pu = Li/Lb;
Ri_pu = Li_pu*0.01; % Ri = 1% of Xi
Ri    = Ri_pu*Zb;
x = 0.1;
Cf = x*Cb; %Filter capacitor
Cf_pu = x;
% Calculation of wres,resonance frequency of the filter
wres = sqrt((Li+Ltran)/(Li*Ltran*Cf));
fres=wres/(2*pi);
%Damping resistance
Rd = 1/(3*wres*Cf);

% Measuring Filter Parameters
% Second order LPF
fc_lpf = fsample/10;
wc_lpf  = 2 * pi * fc_lpf;  wc_lpf2  = wc_lpf * wc_lpf;
% Bessel Filter
a1_lpf = 1.3601/wc_lpf; a2_lpf = 0.6165/wc_lpf2;
% Butterworth Filter Parameters
% a1_lpf = 1.4142/wc_lpf; a2_lpf = 1/wc_lpf2;

% ==== PLL Design
% f0 = 50;           % rated frequency (Hz)
% w0 = 2*pi*f0;  
A0 = 1;         % input signal in pu

zeta = 1/sqrt(2);              % damping ratio (0.5 - 1)
ts   = 0.2;                    % settling time (s)
tau  = ts/4.6;                 % response time constant
wn   = 1/(tau*zeta);           % natural frequency
KP   = 2*zeta*wn;
Ti   = 2*zeta/wn;
Kp   = 2/A0 * KP;              % P gain
Ki   = Kp/Ti;                  % I gain

wf     = 2*zeta*w0;
k = wf/w0;

fc = 20;               % cut-off frequency (Hz)
wc = 2*pi*fc; 
eta = 1/sqrt(2);      % damping ratio (0.5 - 1)
bf = 2 * eta * wc;
cf = wc * wc;

% ====Inner Current Control
Lgrid_pu = Lgrid_lv/Lb;
Rgrid_pu = Rgrid_lv/Zb;

Lseries_pu = Li_pu + Ltran/Lb + Lgrid_pu;  % Total inductance should be used
Rseries_pu = Ri_pu + Rtran/Zb + Rgrid_pu;  % for stable operation in weak grid

Trise_conv = 0.0025;              % Converter current control rise time, 2.5 ms
BW_conv    = log(9)/Trise_conv;   % Bandwith - Trise relation of first-order system 
Tconv = 1/BW_conv;                % Converter time constant
Kpi        = (BW_conv/w0) * Lseries_pu;
Kii        = (BW_conv/w0) * Rseries_pu * w0; %with multiplying by w0, we can use with actual time
Vc_prime   = 0.25;

% ====Outer Control
% Reactive Power Control
Trise_Q    = 0.15;                % Reactive power control rise time, 150 ms
BW_Q       = log(9)/Trise_Q;      % Bandwith - Trise relation of first-order system 
Kpq        = BW_Q * Tconv;
Kiq        = BW_Q;                
Ir_limit   = 1;
Qinit      = 0;
Ir_init    = Qinit;

% % AC Voltage Control
Trise_V  = 0.15;                                  % Reactive power control rise time, 150 ms
BW_V     = log(9)/Trise_V;                        % Bandwith - Trise relation of first-order system, 
Kpv      = (BW_V * Tconv)/(Ltran/Lb + Lgrid_pu);  % dV = (Lg_pu + Lgrid_pu)*dI
Kiv      = BW_V/(Ltran/Lb + Lgrid_pu);                                 
Vinit = 1;
tVinit = 1;


% Active Power Control
Trise_P  = 0.15;                % Active power control rise time, 200 ms
BW_P     = log(9)/Trise_P;      % Bandwith - Trise relation of first-order system 
Kpp      = BW_P * Tconv;
Kip      = BW_P;                
Ia_limit = 1;
Pinit    = 0.85*Pinv/Sb;
Ia_init  = Pinit;

% 
% % DC Voltage control
H_Cdc = 0.1;                          % in J/VA
E_Cdc = H_Cdc * Sb;                   % Energy stored in the DC capacitor (Joule)
Cdc   = 2 * E_Cdc / Vdc / Vdc;        % DC capacitor (F)
% 
Trise_vdc = 0.1;                      % Vdc control rise time, 100 ms
tau_vdc  = Trise_vdc*1.8;             % response time of the second order system
zeta_vdc = 1/sqrt(2);                 % damping ratio (0.5 - 1)
wn_vdc   = 1/(tau_vdc*zeta_vdc);      % natural frequency
KP_vdc   = 2*zeta_vdc*wn_vdc;
Ti_vdc   = 2*zeta_vdc/wn_vdc;
Kp_vdc   = (2*H_Cdc)* KP_vdc;         % P gain
Ki_vdc   = Kp_vdc/Ti_vdc;             % I gain

Ia_limit   = 1;
Pinit      = 0;
Ia_init    = Pinit;

E_ltl = Einv_ltl;
ma = E_ltl*sqrt(2)/Vdc;
phia = 0; %asin((Pinit*Sb)*(Li+Lg+Lgrid)*w0/E_ltl/Einv_ltl);
tinit = 0.25;
dtsim = 5.0e-6;

% Active power reference changes
dP1 = 0.3;
tP1 = 0.5;
dP2 = 0.5;
tP2 = 1.5;
dP3 = 0.8;
tP3 = 2.5;
dP4 = -0.6;
tP4 = 3.5;

% % Reactive power reference changes
% dQ1 = 0;
% tQ1 = 3;
% dQ2 = 0;
% tQ2 = 4;
% dQ3 = 0;
% tQ3 = 5;
% dQ4 = 0;
% tQ4 = 6;

FRT_ON = 0.1;
FRT_OFF = 0.075;
FRT_Time = 0.2;


%% Parameters for Dual Active Bridge Converter
%
Fsw= 10e3;         % PWM switching frequency (Hz)
DT= 150e-9;         % Dead time (s)
Scope_Decimation=1; % Scope Decimation
Ts=1e-7;            % Control system sample time (s)
%
Pnom= 1000;          % Nominal power (W)
Vbat= 25;        % DC source 1 nominal voltage (V)

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
Turn_ratio=15;    % Primary to Secondary turn ratio       
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