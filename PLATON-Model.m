% Vehicle parameters and constants

clear;
clf;

rho = 1.184;  % under standard conditions 20°C 1013.25 mBar %25°C?!!! when T=20°C ρ = 1.204 kg/m3 (Feng)
%rho = 1.292;  % under norm conditions 0°C 1013.25 mBar
C_rr = 0.01;  % tyre on asphalt 
%C_rr = 0.015; % tyre on concrete 
%C_rr = 0.02;  % tyre on gravel
A_f = 8.4 ;   % Vehicle frontal area in m²
v  = 0;       % Initial vehicle speed
dt = 1;       % 1 equal to 0.1 s
C_d = 0.82;   % Drag coefficient 0.82 (long cylinder)
g  = 9.81;    % gravity in m/s²
G  = 1 ;      % Gear =  3(1st)  2(2nd) 1.5(3rd) 1(4th) 0.8(5th) 0.5(6th) 
r_w = 0.280;  % Wheel radius in m
m_empty = 12500 ; 
m_load  =  4240 ;   % fully loaded 53 passengers a 80 kg

%r road conditions
phi= 0; % 0..10 % grade <-> 0..5.7 degrees 

s = 0;
v = 0;
v_dc_int=0;
APP = 0;     % Accelerator pedal position 
BRP = 0;     % Brake pedal position

% Motor parameters
k_i       = 0.0167;     % Motor loss constant 
k_c       = 0.0452;     % Motor loss constant 
k_omega   = 5.0664e-05; % kgm²
C         = 628.2974;   % Motor loss constant from reference motor
MaxOmega  =    750;     % Maximum motor rotational speed to be scaled rpm
MinOmega  =     30;     % Minimum motor rotational speed to be scaled rpm
MaxTorque =    500;     % Maximum torque of motor to be scaled Nm
MaxPower  = 220000;     % 220 kW

F_b=0;
T_loss=0;

% Battery
Capacity      = 130000;  % Wh
Accessoryload = 100;     % e.g. load for HVAC etc. in W
E             = Capacity;
SOC           = 100;     % Initial State of charge in %
V_oc          = 400;     % Voltage open circuit
R_int         = 25/1000; % Internal resistance 80 mOhm

% SORT heavy cycle
SORT_v=zeros(1,1600);
SORT_H = 0;
for i=1:1:1600
 if     i <   50 SORT_H=SORT_H+0.103;  % acceleration 1.02  m/s²
 elseif i <  190 SORT_H=SORT_H+0;      % coast
 elseif i <  250 SORT_H=SORT_H-0.08;   % deceleration  0.8  m/s²
 elseif i <  450 SORT_H=0;             % stop
 elseif i <  550 SORT_H=SORT_H+0.077;  % acceleration 0.77  m/s
 elseif i <  700 SORT_H=SORT_H+0;      % coast
 elseif i <  790 SORT_H=SORT_H-0.08;   % deceleration  0.8  m/s²
 elseif i <  990 SORT_H=0;             % stop 
 elseif i < 1150 SORT_H=SORT_H+0.062;  % acceleration  0.62 m/s
 elseif i < 1250 SORT_H=SORT_H+0.0;    % coast
 elseif i < 1375 SORT_H=SORT_H-0.08;   % deceleration  0.8  m/s²
 else;
 end;
 SORT_v(i) = SORT_H;
end;

# Bus mass
m= m_empty + m_load;
m_i = 1.04*m ;

l=0; % simulation step is 0.1 s dt = 1 equals to time increment of 0.1 s
          % dt*0.1 must be multiplied

% Arrays for variables           
plot_o   =zeros(1,1600);    % create an all-zero matrix 1X1600
plot_tmot=zeros(1,1600);
plot_fi  =zeros(1,1600);
plot_ftr =zeros(1,1600);
plot_a   =zeros(1,1600);
plot_v   =zeros(1,1600);
plot_s   =zeros(1,1600);
plot_i   =zeros(1,1600);
plot_vdc =zeros(1,1600);
plot_soc =zeros(1,1600);

h=waitbar(0);
% Simulation loop

l=0;
rpm=1000;   % start motor speed in revolutions per minute

while l < 1600
l=l+1;
waitbar(l/1600,h);  % expand waitbar h to the new position-- l/1600

omega = 2*pi* rpm / 60;  % rad/s Palstance

% Driver model PID
if (l > 2 ) 
 if (SORT_v(l)  > SORT_v(l-1)) APP=1; BRP =0; end;
 if (SORT_v(l) == SORT_v(l-1)) APP=0; BRP =0; end;
 if (SORT_v(l)  < SORT_v(l-1)) APP=0; BRP =1; end;
end;
            
% Rotation speed omega envelope
if  (omega > MaxOmega ) omega = MaxOmega; end;
if  (omega < MinOmega ) omega = MinOmega; end;

drive_cycle_speed = SORT_v(l) ; % what does cycle speed stand for?
v_dc=drive_cycle_speed-v;  % initial vehicle speed v=0 % what is v_dc ?
v_dc_int=v_dc_int+v_dc*dt*0.1; % initial value of v_dc_int=0 % what is v_dc_int?

Ftr_command = 7500 * v_dc + 1*(v_dc_int); % 7500? Ftr_command ??

% Motor model

% accelerator/brake pedal position from driver
if (APP==1) && (BRP==0) rpm = rpm + 100; end;
if (APP==0) && (BRP==0) rpm = rpm +   0; end;
if (APP==0) && (BRP==1) rpm = rpm - 100; end;


% Torque envelope limiter
PositiveTorque = MaxPower/omega;
RegenerativeTorque = 0; % regenerative energy 
MotorNetTorque = PositiveTorque-RegenerativeTorque;
if MotorNetTorque < MaxTorque T_mot= MotorNetTorque;
else                          T_mot= MaxTorque; end;
P_loss = k_c*T_mot^2 + k_i*omega + k_omega*omega^3 + C;
P_mot  = T_mot*omega - P_loss;     % ?? 
MotorPowerInput = T_mot*omega + P_loss;  % ?? does T_mot*omega include P_loss??

% Driveline
% Gear
if     (v <  3.6) G=  3;  % 1st
elseif (v <  7.2) G=  2;  % 2nd
elseif (v < 11.1) G=1.5;  % 3rd
elseif (v < 13.8) G=1.0;  % 4rd
else              G=0.8;  % 5rd
end;
% Brakes
if   (BRP == 1) F_b = -Ftr_command;
else            F_b = 0;  end;

%F_tr= (T_mot - T_loss)*G/r_w - F_b;
F_tr= Ftr_command;   % Tractive force is commanded directly

if (v != 0) rpm = abs(v)*G/r_w; end;

% Battery model
MotorPowerInput = Ftr_command; % is Ftr_command the tractive force or power?
Pel_out= Accessoryload+MotorPowerInput; % required power for motor and accessoryload
Pel_batt= Pel_out;
I = (V_oc - sqrt(V_oc^2 -4*R_int*Pel_batt))/(2*R_int);
Pel_ideal = I * V_oc;
Pel_loss = I*I*R_int;
Pel_actual = Pel_ideal - Pel_loss;
V_term =V_oc - I* R_int;
E = E - I*V_oc*dt*0.1/3600; % E in Wh  -term: 10 steps = 1Ws -> 1/3600 Wh  
SOC = 100*(E/Capacity); 

% Vehicle (Glider) model
F_aero =  C_d * A_f * rho * v^2 ; % kgm/s² = N
F_rr = m * g * C_rr;
F_grade = m * g * sin(phi);
F_i = F_tr - F_aero - F_rr- F_grade ;
a = F_i/m_i;
v = v + a*dt*0.1; % vehicle speed m/s
s = s + v*dt*0.1; % vehicle travelled distance m

% Fill arrays
plot_o(1,l)=omega; % 1 and L
plot_tmot(1,l)=T_mot;
plot_fi(1,l)=F_i;
plot_ftr(1,l)=F_tr;
plot_a(1,l)=a;
plot_v(1,l)=v;
plot_s(1,l)=s;
plot_I(1,l)=I;
plot_vdc(1,l)=v_dc;
plot_soc(1,l)=SOC;

end ;
close(h);

%f1=figure;
%f2=figure;
%figure(f1);
% Plot result arrays
plot(SORT_v);title("SORT Heavy cycle"); hold on;% hold on means keep the new and old plot in the same figure
%plot(plot_o);
%plot(plot_tmot);title("T_m_o_t");
%plot(plot_fi);title("F_i");
%figure(f2);
%plot(plot_ftr);title("F_t_r");
%plot(plot_a);title("a");
plot(plot_v,'g');title("v");
%plot(plot_s);title("s");
plot(plot_I,'r');
%plot(plot_vdc);
plot(plot_soc,"c");
grid;
