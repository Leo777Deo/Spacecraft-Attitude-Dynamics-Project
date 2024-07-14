%% ------------------------GROUP 51----------------

% Barbiera Andrea
% 
% De Luca Leo
% 
% Perusini Gianluca
% 
% Poverini Viola

% Indications: by pressing Run all the simulation will be run, first the De
% Tumbling, second the Slew-Monoeuvre, third the Earth Pointing.

% The post-processing in terms of figures and computation of the Absolute
% Pointing Error (APE) and Relative Pointing Error (RPE) is elaborated in 
% the section Analysis after the Earth-Pointing.

% All the data and the matching conditions for the Simulink models are 
% already defined in this one script.

clear all; 
clc;
close all;

%% ----------Orbit Parameters------------

% Earth parameters
Re=astroConstants(23); %km
mu=astroConstants(13); % [km^3 / s^2]
J2=astroConstants(9); 
we=15.04*(2*pi/3600);
we=we/3600; % rad/s

% Characterising the orbit for the s/c
a=7259; % km
e=0.001; 
i=deg2rad(97.27276); % inclination (rad)
OM=deg2rad(90); % right ascension of the ascending node (rad)
w=deg2rad(0); % argument of pericentre (rad)
f0=deg2rad(0); % true anomaly (rad)
[r0,v0]=kep2car(a,e,i,OM,w,f0,mu);
n=sqrt(mu/ a^3); % mean angular velocity (rad/s)
T=(2*pi)/n; % Period (s)
rp=a*(1-e); % radius of pericentre (km)
Altitude=rp-Re; % Altitude (km)

%Apparent Sun orbit:
n_sun=2*pi/(365.26*24*3600); % mean angular velocity of the sun (rad/s)
eps_sun=deg2rad(23.45); % inclination of the sun (rad)
e_sun=0.0167086; 
a_sun=149.60e6;

% Orbit representation;
s0=[r0;v0]; % i.c.
op=odeset('RelTol',1e-12,'AbsTol',1e-14);
[t,r_orb]=ode113(@(t,y) ode_2bp(t,y,mu),[0 T],s0,op);

% Orbit
figure(1)
plot3(r_orb(:,1),r_orb(:,2),r_orb(:,3));
Earth;
quiver3(0,0,0,9000,0,0);
quiver3(0,0,0,0,9000,0);
quiver3(0,0,0,0,0,9000);


%% ------S/c characteristics------

Mass_sc=75; %kg
x_sc=0.5; %m
y_sc=0.47; %m
z_sc=0.36; %m

Volume_sc=x_sc*y_sc*z_sc; % m^3

% Inerita matrix
Ix=(1/12) * Mass_sc *(y_sc^2 + z_sc^2); %kg*m^2
Iy=(1/12) * Mass_sc *(x_sc^2 + z_sc^2); %kg*m^2
Iz=(1/12) * Mass_sc *(y_sc^2 + x_sc^2); %kg*m^2
I=[Ix;Iy;Iz];
I=diag(I);
I_inv=I\eye(3,3);

% Faces  
r1=[y_sc/2;0;0]; % m
r2=[0;x_sc/2;0];
r3=[0;0;z_sc/2];
r4=[-y_sc/2;0;0];
r5=[0;-x_sc/2;0];
r6=[0;0;-z_sc/2];

A1=x_sc*z_sc; %m^2
A2=y_sc*z_sc;
A3=x_sc*y_sc;
A4=A1;
A5=A2;
A6=A3;

%% -------Initial Conditions--------

% Angular velocity i.c.
wx_D=deg2rad(2); %rad/s
wy_D=deg2rad(2); %rad/s
wz_D=deg2rad(2); %rad/s
w0_D=[wx_D;wy_D;wz_D];

% Euler angles i.c.

phi312_D=(2/3)*pi; %rad
theta312_D=(2/3)*pi;
psi312_D=(2/3)*pi;

IC312_D=[phi312_D;theta312_D;psi312_D];

halfrange=pi/40; % switch condition for Euler angles

%% ------Perturbations-------

% Gravity Gradient and Magnetic field are the main perturbations:

%Magnetic field
jb=[0.1; 0.1;0.1]; % extreme case for induction currents (amp^2)
eps_b=11.5*pi/180; % inclination of B wrt E's pole
g_1_0=-29404.8; %nT
g_1_1=-1450.9; 
h_1_1=4653.5; 

%% --------Sensors------

samp_freq=1; %Hz
samp_t=1/samp_freq; %s
n_t=samp_t; % noise time

%Gyro:
arw=0.15; %deg
ARW=(((arw*pi/180)/(sqrt(n_t)))^2)*n_t;
rrw=0.0003; %deg
RRW=(((rrw*pi/180)/(sqrt(n_t)))^2)*n_t;

teta_g_x=0.5*pi/180; % misalignment of 0.5 degrees (rad)
teta_g_y=0.5*pi/180;
teta_g_z=0.5*pi/180;
Miss_G=[teta_g_x;teta_g_y;teta_g_z]; % Mounting error vector
Mo_Gideal=eye(3,3); %Mounting direction of the sensor

Gain_G=eye(3,3); %Gain of Gyroscopes
Gs_res=deg2rad(1/60); %rad/s 

% Horizon sensor:
Hs_res=deg2rad(0.15); %Horizon sensor resolution (rad)
Hs_accuracy=deg2rad(1/80); %Horizon sensor accuracy (rad)
Hs_var=Hs_accuracy^2; % Variance

teta_h_x=deg2rad(0.5);
teta_h_y=deg2rad(0.5);
teta_h_z=deg2rad(0.5);
Miss_H=[teta_h_x;teta_h_y;teta_h_z]; % Mounting error (misalignment)

% Sun sensor:
Ss_res=deg2rad(0.06); % Sun sensor resolution (rad)
Ss_accuracy=deg2rad(1/80); % Sun sensor accuracy (rad^2)
Ss_var=Hs_accuracy^2; % Variance 

teta_s_x=deg2rad(0.5);
teta_s_y=deg2rad(0.5);
teta_s_z=deg2rad(0.5);
Miss_S=[teta_s_x;teta_s_y;teta_s_z]; % Mounting error (misalignment)

%% ------Control-----------

w_t_D=zeros(3,1); %rad/s (target ang vel for De-tumbling)

% Coefficients for the control
kdx_D=0.9896;
kdy_D=0.9734;
kdz_D=0.9096; 

%% ------Actuator:CMG-------

a_pyr=1; % length of the pyramid
b_pyr=a_pyr; % base of the pyramid
h_pyr=a_pyr; % height of the pyramid
v_pyr=[a_pyr;b_pyr;h_pyr]; % square pyramid configuration
vpyr=norm(v_pyr);
uv_pyr=v_pyr/vpyr;
Mo_CMG=[-uv_pyr(1,1) uv_pyr(1,1) uv_pyr(1,1) -uv_pyr(1,1);
        -uv_pyr(2,1) -uv_pyr(2,1) uv_pyr(2,1) uv_pyr(2,1);
          uv_pyr(3,1) uv_pyr(3,1) uv_pyr(3,1) uv_pyr(3,1)]; % Mounting of CMGS

Beta_CMG=acos(dot([1;0;0],uv_pyr)); % angle of the pyramid (rad)

delta_0_D=zeros(4,1); % I.C. for gimbal angle
i_r=0.05; %kg m^2
I_r=diag(i_r*ones(4,1));
I_r_inv=I_r\eye(4,4);

h_r=[0.5;0.5;0.5;0.5];
h_r_m=norm(h_r); %modulus of h_r

Mc_max_disp=1.1; %Nm

k_cmg=-1; % coefficeint for null control

%% -----SIM options---------

t0=0;
tf_D=50; % s (simulation time for De-tumbling)
Fixed_Step_D=0.01;

sim_options.SolverType = 'Fixed-step';      
sim_options.Solver ='ode8';                
sim_options.FixedStep = 'Fixed_Step_D';              
sim_options.StartTime = '0';               
sim_options.StopTime = 'tf_D';  
sim_options.RelTol='1e-6';
    
D=sim('Detumbling.slx',sim_options);

%% ----------Slew Manoeuvre--------

% All the matching conditions between the terminating values of the
% De-Tumbling will be defined as initial conditions for the Slew-Manoeuvre.

% After the deployment of the panels the inertia properties of the
% satellite have changed.

%% ----------Orbit parameter-----------

f0_SM=D.Theta_D(end,1); % Matching true anomaly
t_sun_SM=tf_D; % matching time

%% -------Inertia after solar panels deployment----------

% Panels (m)

l_p=0.7; %m
b_p=0.6;
d_p=(y_sc/2)+(l_p/2);

rp1=[0;d_p;0];
rp2=[0;-d_p;0];
Ap1=l_p*b_p;
Ap2=Ap1;

n_p=[0;0;1]; % unit vector that represents the normal to the panel

% Inertia matrix after the panels are deployed

m_p1=3.8*Ap1; % mass of the panels computed as 3.8 kg/m^2
m_p2=3.8*Ap2;

Ip1_x=(1/12)*m_p1*(l_p^2) + m_p1*d_p^2;
Ip1_y=(1/12)*m_p1*(b_p^2) + m_p1*d_p^2;
Ip1_z=(1/12)*m_p1*(l_p^2 + b_p^2) + m_p1*d_p^2;

Ip2_x=(1/12)*m_p2*(l_p^2) + m_p2*d_p^2;
Ip2_y=(1/12)*m_p2*(b_p^2) + m_p2*d_p^2;
Ip2_z=(1/12)*m_p2*(l_p^2 + b_p^2) + m_p2*d_p^2;

ip_1=[Ip1_x;Ip1_y;Ip1_z];
ip_2=[Ip2_x;Ip2_y;Ip2_z];

Ip_1_transp=diag(ip_1);
Ip_2_transp=diag(ip_2);

I_tot=I+Ip_1_transp+Ip_2_transp;

I_tot_inv=I_tot\eye(3,3);

%% -------I.C.--------

wx_SM=D.w_b_D(end,1); %rad/s
wy_SM=D.w_b_D(end,2); %rad/s
wz_SM=D.w_b_D(end,3); %rad/s
w0_SM=[wx_SM;wy_SM;wz_SM];

% Euler angles i.c.

if D.TF_D(:,:,end)==1

phi312_SM=D.Ang_312_D(end,1);
theta312_SM=D.Ang_312_D(end,2);
psi312_SM=D.Ang_312_D(end,3);

IC312_SM=[phi312_SM;theta312_SM;psi312_SM];

elseif D.TF_D(:,:,end)==0 

phi312_SM=D.Ang_313_into_312_D(end,1);
theta312_SM=D.Ang_313_into_312_D(end,2);
psi312_SM=D.Ang_313_into_312_D(end,3);

IC312_SM=[phi312_SM;theta312_SM;psi312_SM];    
end

% The Euler Angles are checked to see whether or not the terminating
% orientation matrix stemmed from 313 or 312 angles and the the i.c. 
% for the Slew-Manoeuvre are imposed.

%% ------Control-------

% Coefficients for the control
kdx_SM=0.534;
kdy_SM=1.198;
kdz_SM=1.2;
kpx_SM=0.129*0.5;
kpy_SM=0.31*0.5;
kpz_SM=0.873*0.5;

%% -----Actuators------

delta_0_SM=D.delta_g_D(end,:); % matching the actuators' gimbal motion.

%% -----SIM options--------

t0=0;
tf_SM=500; % s (simulation time for Slew-Manoeuvre)
Fixed_Step_SM=0.01;

sim_options.SolverType = 'Fixed-step';      
sim_options.Solver ='ode8';                
sim_options.FixedStep = 'Fixed_Step_SM';              
sim_options.StartTime = '0';               
sim_options.StopTime = 'tf_SM';  
sim_options.RelTol='1e-6';
    
SM=sim('Slew_Manoeuvre.slx',sim_options);


%% --------Pointing----------

% All the matching conditions between the terminating values of the
% Slew-Manoeuvre will be defined as initial conditions for the Pointing.

%% ---------Orbit Parameters--------

f0_P=SM.Theta_SM(end,1);
t_sun_P=tf_D+tf_SM;

%% --------I.C.------------

wx_P=SM.w_b_SM(end,1); %rad/s
wy_P=SM.w_b_SM(end,2); %rad/s
wz_P=SM.w_b_SM(end,3); %rad/s
w0_P=[wx_P;wy_P;wz_P];

if SM.TF_SM(:,:,end)==1

phi312_P=SM.Ang_312_SM(end,1);
theta312_P=SM.Ang_312_SM(end,2);
psi312_P=P.Ang_312_P(end,3);

IC312_P=[phi312_P;theta312_P;psi312_P];

elseif SM.TF_SM(:,:,end)==0 

phi312_P=SM.Ang_313_into_312_SM(end,1);
theta312_P=SM.Ang_313_into_312_SM(end,2);
psi312_P=SM.Ang_313_into_312_SM(end,3);

IC312_P=[phi312_P;theta312_P;psi312_P];    
end

%% ------Control-------

% Coefficients for control
kdx_P=2;
kdy_P=2.365;
kdz_P=2.1;
kpx_P=1.154*0.5;
kpy_P=1.4*0.5;
kpz_P=1*0.5;

%% -----Actuators------

delta_0_P=SM.delta_g_SM(end,:);

%% ------Sim options-------

t0=0;
tf_P=T; % s (simulation time for Pointing)
Fixed_Step_P=0.01;

sim_options.SolverType = 'Fixed-step';      
sim_options.Solver ='ode8';                
sim_options.FixedStep = 'Fixed_Step_P';              
sim_options.StartTime = '0';               
sim_options.StopTime = 'tf_P';  
sim_options.RelTol='1e-6';
    
NP=sim('Nadir_Pointing.slx',sim_options);

%% --------Analysis----------

t_s_SM=1:length(SM.t_s_SM);

for i=1:length(t_s_SM)
    detA_err_SM(i,1)=det(SM.A_err_SM(:,:,i));
end

t_s_P=1:length(NP.t_s_P);

for i=1:length(t_s_P)
    detA_err_P(i,1)=det(NP.A_err_P(:,:,i));
end

% Definition of dummy time vectors to patch all graphs together
t_patch_SM=[D.t_D(end,1):Fixed_Step_SM:(tf_D+tf_SM)];
t_c_patch_SM=[D.t_D(end,1):samp_t:(tf_D+tf_SM)];

t_patch_P=[(SM.t_SM(end,1)+D.t_D(end,1)):Fixed_Step_P:(tf_D+tf_SM+tf_P)];
t_c_patch_P=[(SM.t_SM(end,1)+D.t_D(end,1)):samp_t:(tf_D+tf_SM+tf_P)];

%% ------------Plot---------------

% Angular velocity evolution:
figure(2)
plot(D.t_D,rad2deg(D.w_b_D(:,1)),'r');
hold on;
plot(t_patch_SM,rad2deg(SM.w_b_SM(:,1)),'r');
hold on;
plot(t_patch_P,rad2deg(NP.w_b_P(:,1)),'r');
hold on;
plot(D.t_D,rad2deg(D.w_b_D(:,2)),'b');
hold on;
plot(t_patch_SM,rad2deg(SM.w_b_SM(:,2)),'b');
hold on;
plot(t_patch_P,rad2deg(NP.w_b_P(:,2)),'b');
hold on;
plot(D.t_D,rad2deg(D.w_b_D(:,3)),'g');
hold on;
plot(t_patch_SM,rad2deg(SM.w_b_SM(:,3)),'g');
hold on;
plot(t_patch_P,rad2deg(NP.w_b_P(:,3)),'g');
xline([tf_D tf_SM+tf_D],'--k');
legend('wx','','','wy','','','wz');
title('$Angular$ $Velocity$','Interpreter','latex');
xlabel('$t$ $[s]$','Interpreter','latex');
ylabel('$w$ $[deg/s]$','Interpreter','latex');
ylim([-3 3]);
xlim([0 tf_D+tf_SM+tf_P+195]);

% Angular Velocity during De-Tumbling
figure(3)
plot(D.t_D,rad2deg(D.w_b_D(:,1)),'r');
hold on;
plot(D.t_D,rad2deg(D.w_b_D(:,2)),'b');
hold on;
plot(D.t_D,rad2deg(D.w_b_D(:,3)),'g');
yline(0,'--k');
legend('wx','wy','wz');
title('$Angular$ $Velocity$ $during$ $De$ $-$ $Tumbling$','Interpreter','latex');
xlabel('$t$ $[s]$','Interpreter','latex');
ylabel('$w$  $[deg/s]$','Interpreter','latex');

% Error matrix defined in the Simulink models

% Evolution of Extra-Diagonal terms in Error Matrix
figure(4)
plot(t_c_patch_SM,rad2deg(SM.alpha_err_SM(:,1)),'r');
hold on;
plot(t_c_patch_P,rad2deg(NP.alpha_err_P(:,1)),'r');
hold on;
plot(t_c_patch_SM,rad2deg(SM.alpha_err_SM(:,2)),'b');
hold on;
plot(t_c_patch_P,rad2deg(NP.alpha_err_P(:,2)),'b');
hold on;
plot(t_c_patch_SM,rad2deg(SM.alpha_err_SM(:,3)),'g');
hold on;
plot(t_c_patch_P,rad2deg(NP.alpha_err_P(:,3)),'g');
hold on;
plot(t_c_patch_SM,rad2deg(SM.alpha_err_SM(:,4)),'Color','#A2142F');
hold on;
plot(t_c_patch_P,rad2deg(NP.alpha_err_P(:,4)),'Color','#A2142F');
hold on;
plot(t_c_patch_SM,rad2deg(SM.alpha_err_SM(:,5)),'c');
hold on;
plot(t_c_patch_P,rad2deg(NP.alpha_err_P(:,5)),'c');
hold on; 
plot(t_c_patch_SM,rad2deg(SM.alpha_err_SM(:,6)),'Color','#77AC30');
hold on;
plot(t_c_patch_P,rad2deg(NP.alpha_err_P(:,6)),'Color','#77AC30');
xline([tf_D tf_SM+tf_D],'--k');
legend('Alpha 23','','Alpha 12','','Alpha 13','','Alpha 32','','Alpha 21','','Alpha 31','');
title('$Extra$ $Diagonal$ $Components$ $of$ $Error$ $Matrix$','Interpreter','latex');
xlabel('$t$ $[s]$','Interpreter','latex');
ylabel('$Extra$ $Diagonal$ $Components$ $[deg]$','Interpreter','latex');
ylim([-20 20]);
xlim([0 tf_D+tf_SM+tf_P+195]);

% Evolution of Diagonal terms in Error Matrix
figure(5)
plot(t_c_patch_SM,SM.Diag_A_err_SM(:,1),'r');
hold on;
plot(t_c_patch_P,NP.Diag_A_err_P(:,1),'r');
hold on;
plot(t_c_patch_SM,SM.Diag_A_err_SM(:,2),'b');
hold on;
plot(t_c_patch_P,NP.Diag_A_err_P(:,2),'b');
hold on;
plot(t_c_patch_SM,SM.Diag_A_err_SM(:,3),'g');
hold on;
plot(t_c_patch_P,NP.Diag_A_err_P(:,3),'g');
xline([tf_D tf_SM+tf_D],'--k');
legend('A err 11','','A err 22','','A err 33');
title('$Diagonal$ $Components$ $of$ $Error$ $Matrix$','Interpreter','latex');
xlabel('$t$ $[s]$','Interpreter','latex');
ylabel('$Diagonal$  $Components$ $[-]$','Interpreter','latex');
ylim([-0.4 1.7]);
xlim([0 tf_D+tf_SM+tf_P+195]);

% Evolution of the determinant of Error Matrix
figure(6)
plot(t_c_patch_SM,detA_err_SM,'r');
hold on;
plot(t_c_patch_P,detA_err_P,'r');
hold on;
xline([tf_D tf_SM+tf_D],'--k');
title('$Determinant$ $of$ $Error$ $Matrix$','Interpreter','latex');
xlabel('$t$ $[s]$','Interpreter','latex');
ylabel('$Determinant$ $[-]$','Interpreter','latex');
ylim([0.99999999995 1.00000000005]);

% Evolution of the torque provided by the CMGs
figure(7)
plot(D.t_D,D.Mc_eff_D(:,1),'r');
hold on;
plot(t_patch_SM,SM.Mc_eff_SM(:,1),'r');
hold on;
plot(t_patch_P,NP.Mc_eff_P(:,1),'r');
hold on;
plot(D.t_D,D.Mc_eff_D(:,2),'b');
hold on;
plot(t_patch_SM,SM.Mc_eff_SM(:,2),'b');
hold on;
plot(t_patch_P,NP.Mc_eff_P(:,2),'b');
hold on;
plot(D.t_D,D.Mc_eff_D(:,3),'g');
hold on;
plot(t_patch_SM,SM.Mc_eff_SM(:,3),'g');
hold on;
plot(t_patch_P,NP.Mc_eff_P(:,3),'g');
xline([tf_D tf_SM+tf_D],'--k');
legend('Mx CMG','','','My CMG','','','Mz CMG','','');
title('$Torque$ $CMG$','Interpreter','latex');
xlabel('$t$ $[s]$','Interpreter','latex');
ylabel('$M$ $eff$ $[N m]$','Interpreter','latex');
xlim([0 tf_D+tf_SM+tf_P+195]);

% Pointing Error
figure(8)
plot(D.t_D,rad2deg(D.P_err_D),'Color','#D95319');
hold on;
plot(t_patch_SM,rad2deg(SM.P_err_SM),'Color','#D95319');
hold on;
plot(t_patch_P,rad2deg(NP.P_err_P),'Color','#D95319');
hold on;
xline([tf_D tf_SM+tf_D],'--k');
title('$Pointing$ $Error$','Interpreter','latex');
xlabel('$t$ $[s]$','Interpreter','latex');
ylabel('$Pointing$ $Error$ $[deg]$','Interpreter','latex');
ylim([0 10]);
xlim([0 tf_D+tf_SM+tf_P+195]);

%% -------------APE and RPE-------------

M_P_err=mean(rad2deg(NP.P_err_P)); % mean value
V_P_err=var(rad2deg(NP.P_err_P)); % variance
S_P_err=std(rad2deg(NP.P_err_P)); % standard deviation

% Confidence Intervals at different levels
CI_P_err1=0.683 * (S_P_err / sqrt(length(NP.P_err_P))); % 68.3%
CI_P_err2=0.955 * (S_P_err / sqrt(length(NP.P_err_P))); % 95.5%
CI_P_err3=0.997 * (S_P_err / sqrt(length(NP.P_err_P))); % 99.7%

P_req_h=1; % deg (upper bound of the pointing requirement)
P_req_l=0.1; % deg (lower bound of the pointing requirement)

Abs_P_err=abs(rad2deg(NP.P_err_P) - P_req_l*ones(length(NP.P_err_P),1)); %APE

M_Abs_P_err=mean(Abs_P_err); % MPE (mean pointing error, required for RPE)
V_Abs_P_err=var(Abs_P_err);
S_Abs_P_err=std(Abs_P_err);

CI_P_Abs_err1=0.683 * (S_Abs_P_err / sqrt(length(Abs_P_err)));
CI_P_Abs_err2=0.955 * (S_Abs_P_err / sqrt(length(Abs_P_err)));
CI_P_Abs_err3=0.997 * (S_Abs_P_err / sqrt(length(Abs_P_err)));

R_Abs_P_err=abs(Abs_P_err - M_Abs_P_err*ones(length(NP.P_err_P),1)); %RPE

M_R_P_err=mean(R_Abs_P_err);
V_R_P_err=var(R_Abs_P_err);
S_R_P_err=std(R_Abs_P_err);

CI_P_R_err1=0.683 * (S_R_P_err / sqrt(length(R_Abs_P_err)));
CI_P_R_err2=0.955 * (S_R_P_err / sqrt(length(R_Abs_P_err)));
CI_P_R_err3=0.997 * (S_R_P_err / sqrt(length(R_Abs_P_err)));

% end