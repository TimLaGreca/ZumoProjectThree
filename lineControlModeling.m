%{
Project 3
by Tim LaGreca

Matlab code to model the performance of the zumo robot's line following.
%}

close all;
clear all; 
clc;

%% Section 1 - Determine Zumo's Max Velocity
encoderRate = xlsread('maxSpeedData.xlsx');

micros = encoderRate(:,1);  %units of microseconds
lEncoder = encoderRate(:,2);  %raw reading
rEncoder = encoderRate(:,3);  %raw reading

%convert encoder data to degrees per second
CPR = 50*12;  %units of counts/revs, https://www.pololu.com/docs/0J63/3.4

time = (micros-micros(1,:)).*1e-6;  %units of seconds
% Angular position and position vectors
lRevs = lEncoder/CPR;  %units of revolutions
rRevs = rEncoder/CPR;  %units of revolutions

lTheta = lRevs*(2*pi());  %units of radians
rTheta = rRevs*(2*pi());  %units of radians

%derive angular position, filter, and simulate to get angular velocity
rWheel = 1.551/2*2.54/100;  %units of meters
A = 45;  %educated guess
s1 = tf('s');
G = A/(s1+A);

timeFilter = linspace(time(1), time(end), length(time));
dI = interp1(time, lTheta, timeFilter);
dFilter = lsim(G, dI, timeFilter);
dTheta = diff(dFilter);
dT = diff(timeFilter);

lAngVel = dTheta./dT;  %units of radians per second
lVel = lAngVel*rWheel;  %units of meters per second

% plot for top-down modeling
% figure (1)
% plot(time(1:length(lVel)),lVel);
% xlabel('Time (s)');
% ylabel('Velocity (m/s)');
% title('Velocity v. Time');

maxVel = 0.9;  %m/s, from plot observation

%% Section 2 - Model
vel = maxVel*.5;

U = vel;  %forward speeed
D = 1.997*2.54/100; %distance between front and rear axles, measured and converted to meters
L = D/2 + 0.921*2.54/100;  %preview (lookahead) distance, measured from Zumo's angle of rotation (center of both axels) to line sensors

%handling steps
minLen = 5.5*2.54/100;  %min length to adjust after step, units: m
minTime = minLen/vel;  %min time needed to adjust for step based on speed

%calculating appropriate zeta and wn based on min time
zetaWn = 4/minTime;  %product of zeta and wn based on min time
wn = 18.3;  %appropriate natrual frequency
zeta = 0.992;  %zeta based on wn
ts = 4/(wn*zeta);

%for model performance evaluation... using wn = wd/sqrt(1-zeta^2) and zeta*Wn = 12.8848
Td = minTime/5;  %damped period (observed period) minTime needs to be at SS aka 5 time constants
wd = sqrt(wn^2-(zeta*wn)^2);

s = -1/Td;
a = -s;

%sd
realSd = -zeta*wn
imagSd = wn*sqrt(1-zeta^2)
sd = realSd + imagSd*i;

%angles
angleP1 = atan2(imag(sd),real(sd))
angleP2 = atan2(imag(sd),(real(sd)+a))
alphaZ =  0.5*(-180+angleP1+angleP2)

%zero
x = imag(sd)/tand(alphaZ);
z = abs(real(sd)-x)

%plant P, feedback elements H
s = tf('s');
P = U^2/(D*s^2);  %plant transfer function from steer->lateral position
H = (L*s+U)/U;  %sensor transfer function measuring road lateral position at the lookahead distance

figure(2)
rlocus(P*H)
hold on;
plot(real(sd), imag(sd),'c*')
hold off;

PHmag = abs(U^2/(D*sd^2)*(L*sd+U)/U);

%gains
k = 1/PHmag;
ksum = 1.0;  %ensures stability
kd = k/ksum  %ensures stability
kp = 2*z*kd
ki = z^2*kd

C = ((s+z)*(s+z))/s;
%C = kp + kd*s;

%check position response
Gcl1 = minreal(C*P*H/(1+C*P*H));

figure(3)
step(Gcl1)
ylabel('Amplitude of Response (m)');
title('Line Following PD Step Response');

% G_u = C/(1+C*P*H);

% figure(4)
% step(G_u)
% ylabel('steer angle required (rad)')
 

%% Step Data
stepData = xlsread('stepData.xlsx');

micros = stepData(:,1);  %units of microseconds
eStep = stepData(:,2);  %raw reading
uStep = stepData(:,3);  %raw reading

time = (micros-micros(1,:)).*1e-6;  %units of seconds

timeError = linspace(time(1), time(end), length(time));

s = tf('s');
Cstep = ksum * (kp + kd*s);
Pstep = U^2/(D*s^2);

tfError = 1/(1+Cstep*Pstep);


[ystep, tstep] = step(eStep*tfError,timeError);


figure(4)
plot(time, eStep);
hold on
plot(tstep, ystep)
hold off
xlabel('Time (s)')
ylabel('Error (m)')
title('Error: Modeled vs. Real')
legend('Real Data', 'Modeled Data')

