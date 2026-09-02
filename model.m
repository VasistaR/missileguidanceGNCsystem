clc; close all; clear all;
format short;

A = [-1.064 1.000; 290.26 0.00];
B = [-0.25; -331.40];
C = [-123.34 0.00; 0.00 1.00];
D = [-13.51; 0];

states = {'AoA', 'a'};
inputs = {'\delta_c'};
outputs = {'Az', 'q'};

sys = ss(A,B,C,D,'statename',states,...
    'inputname',inputs,...
    'outputname',outputs);

%TF
TFs = tf(sys);
TF = TFs(2, 1);
disp(pole(TF));

%LQR weight matrices
Q = [0.1 0; 0 0.1];
R = 0.5;

%LQR gain
[K,S,e] = lqr(A,B,Q,R);
fprintf('eigenvalues of A-BK\n');
disp(eig(A-B*K));
fprintf('Feedback gain K');
disp(K)

%closed loop system
Acl = A-B*K;
Bcl = B;

syscl = ss(Acl, Bcl,C,D,'statename',states,...
    'inputname',inputs,...
    'outputname',outputs);

%our TF - closed loop
TF = tf(syscl);
TFc = TF(2,1);

%LQG Kalman filter design
G = eye(2);
H = 0*eye(2);

%Kalman Q, R noise natrices
QBar = diag(0.00015*ones(1,2));
RBar = diag(0.55*ones(1,2));

%defining the noisy system
sys_n = ss(A, [B G], C, [D H]);
[kest,L,P] = kalman(sys_n, QBar, RBar, 0);

%kalman gain observer closed loop
Aob = A-L*C;

%display observer eigenvalues
fprintf('Obvserver eigenvalues\nm');
disp(eig(Aob));

%%noise time constants (you choose)
dT1 = 0.75;
dT2 = 0.25;

%%m missile parameters
R = 6371e3; %earth raidus
Vel = 1021.08; %speed (m/s)
m2f = 3.2811; %meters to feet

%target location
LAT_TARGET = 34.6588;
LON_TARGET = -118.769745;
ELEV_TARGET = 795; %m - MSL

%initial location
LAT_INIT = 34.2329;
LON_INIT = -119.4573;
ELEV_INIT = 10000; %m p- MSL

%obstacle location
LAT_OBS = 34.61916;
LON_OBS = -118.8429;

d2r = pi/180; %converting from degrees to radians

%converting to radians
l1 = LAT_INIT*d2r;
u1 = LON_INIT*d2r;
l2 = LAT_TARGET*d2r;
u2 = LON_TARGET*d2r;

dl = l2-l1;
du = u2-u1;

%haversine formual

a = sin(dl/2)^2 + cos(l1)*cos(l2)*sin(du/2)^2;

c = 2*atan2(sqrt(a),sqrt(1-a));

d = R*c; %horizontal distance (in m)

%initial range (pythagorean theorem)
r = sqrt(d^2+(ELEV_TARGET-ELEV_INIT)^2);

%initial azimuth
yaw_init = azimuth(LAT_INIT,LON_INIT,LAT_TARGET,LON_TARGET);
yaw = yaw_init*d2r;

%% initial flight path angle
dh = abs(ELEV_TARGET-ELEV_INIT);
FPA_INIT = atan(dh/d); %rad