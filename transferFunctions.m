%Setup G(s)
Gs = tf(1,[400 0 0]);

%Setup K1(s),K2(s),K3(s)
K1 = tf([0.1 0.01], [1 0]);
K2 = tf([240 3], [80 20]);
K3 = tf([1400 35],[40 80]);

%Setup time vectors for necessary simulations
t1 = 0:0.5:500; % for input response
t2 = 0:150:172800;% for disturbance input

%Setup disturbance input
d = (1E-4)*cos((7.28E-5)*t2);

%Setup input Step
opt = stepDataOptions('StepAmplitude',5*pi/180);
refPlusDelta = 1.02*5 + 0*t1; %Degrees
refMinusDelta =0.98*5 + 0*t1; %Degrees
refMaxOvershoot =1.25*5 + 0 *t1; %Degrees
distMax = 0.05 + 0*t2; %degrees

%{
System views (All feedback is negative)

R --> K --> G --> Theta
   ^           |
   |-----------|


R ----> K ----> Tc
    ^       |
    |-- G <-|


D ----> G ----> Theta
    ^       |
    |-- K <-|


D --> G --> K --> Tc
   ^           |
   |-----------|
%}

%Prepare Systems for K1
sysfoK1 = series(K1,Gs);
inOutK1 = feedback(sysfoK1,1);
inCtrlK1 = feedback(K1,Gs);
distOutK1 = feedback(Gs,K1);
distCtrlK1 = feedback(sysfoK1,1);

%Prepare outputs for K1
ThetaStepRespK1 = step(inOutK1,t1,opt) * 180/pi;
TcStepRespK1 = step(inCtrlK1,t1,opt);
ThetaDistRespK1 = lsim(distOutK1, d, t2) * 180/pi;
TcDistRespK1 = lsim(distCtrlK1, d, t2);

figure;
plot(t1, ThetaStepRespK1, t1, refPlusDelta, '--', t1, refMinusDelta,'--', t1, refMaxOvershoot, '--')
title('Output Angle Response to 5 degree step System K1');
xlabel('time (seconds)');
ylabel('Theta (degrees)');

figure;
plot(t1,TcStepRespK1);
title('Control Torque response to 5 degree step System K1');
xlabel('time (seconds)');
ylabel('Tc (N*m)');

figure;
plot(t2/3600,ThetaDistRespK1,t2/3600,distMax,'--');
title('Output Angle Response to Disturbance System K1');
xlabel('time (hours)');
ylabel('Theta (degrees');

figure;
plot(t2/3600,TcDistRespK1);
title('Control Torque Response to Disturbance System K1');
xlabel('time (hours)');
ylabel('Tc (N*m)');
xlim([0 48]);


%Prepare Systems for K2
sysfoK2 = series(K2,Gs);
inOutK2 = feedback(sysfoK2,1);
inCtrlK2 = feedback(K2,Gs);
distOutK2 = feedback(Gs,K2);
distCtrlK2 = feedback(sysfoK2,1);

%Prepare outputs for K2
ThetaStepRespK2 = step(inOutK2,t1,opt) * 180/pi;
TcStepRespK2 = step(inCtrlK2,t1,opt);
ThetaDistRespK2 = lsim(distOutK2, d, t2) * 180/pi;
TcDistRespK2 = lsim(distCtrlK2, d, t2);

figure;
plot(t1, ThetaStepRespK2, t1, refPlusDelta, '--', t1, refMinusDelta,'--', t1, refMaxOvershoot, '--')
title('Output Angle Response to 5 degree step System K2');
xlabel('time (seconds)');
ylabel('Theta (degrees)');

figure;
plot(t1,TcStepRespK2);
title('Control Torque response to 5 degree step System K2');
xlabel('time (seconds)');
ylabel('Tc (N*m)');

figure;
plot(t2/3600,ThetaDistRespK2,t2/3600,distMax,'--');
title('Output Angle Response to Disturbance System K2');
xlabel('time (hours)');
ylabel('Theta (degrees');

figure;
plot(t2/3600,TcDistRespK2);
title('Control Torque Response to Disturbance System K2');
xlabel('time (hours)');
ylabel('Tc (N*m)');


%Prepare Systems for K3
sysfoK3 = series(K3,Gs);
inOutK3 = feedback(sysfoK3,1);
inCtrlK3 = feedback(K3,Gs);
distOutK3 = feedback(Gs,K3);
distCtrlK3 = feedback(sysfoK3,1);

%Prepare outputs for K3
ThetaStepRespK3 = step(inOutK3,t1,opt) * 180/pi;
TcStepRespK3 = step(inCtrlK3,t1,opt);
ThetaDistRespK3 = lsim(distOutK3, d, t2) * 180/pi;
TcDistRespK3 = lsim(distCtrlK3, d, t2);

figure;
plot(t1, ThetaStepRespK3, t1, refPlusDelta, '--', t1, refMinusDelta,'--', t1, refMaxOvershoot, '--')
title('Output Angle Response to 5 degree step System K3');
xlabel('time (seconds)');
ylabel('Theta (degrees)');

figure;
plot(t1,TcStepRespK3);
title('Control Torque response to 5 degree step System K3');
xlabel('time (seconds)');
ylabel('Tc (N*m)');

figure;
plot(t2/3600,ThetaDistRespK3,t2/3600,distMax,'--');
title('Output Angle Response to Disturbance System K3');
xlabel('time (hours)');
ylabel('Theta (degrees');

figure;
plot(t2/3600,TcDistRespK3);
title('Control Torque Response to Disturbance System K3');
xlabel('time (hours)');
ylabel('Tc (N*m)');