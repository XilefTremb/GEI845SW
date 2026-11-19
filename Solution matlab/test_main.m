%%%  calculate_zmp.m
%%%  ZMP(IZMP) calculation and display (3.4.1 Derivation of ZMP)
close all
clear
addpath(genpath('Fct kajita'));



global uLINK G M Dtime
G = 9.8;  % Gravity acceleration [m/s^2]
ToDeg = 180/pi;
ToRad = pi/180;
UX = [1 0 0]';
UY = [0 1 0]';
UZ = [0 0 1]';

%%SetupBipedRobot2;   % Biped robot in Fig.2.19, Fig.20 with center of mass and inertia tensor for each link
%% 1) Creer le robot (remplit la variable GLOBALE uLINK, retourne les indices)
ID = SetupDarwinRobot();
shapeBody = [0.178 0.296 0.156];            % [profondeur largeur hauteur]
cBody     = [-0.000235 -0.021006 -0.00913]';% CoM composite du tronc (URDF)
shapeFoot = [0.104 0.066 0.008];            % semelle du Darwin-OP
ankleH    = 0.0335;                         % cheville -> semelle (a confirmer)

for n=1:length(uLINK)
    uLINK(n).ddq = 0;  uLINK(n).I = zeros(3,3);
    uLINK(n).Ir  = 0;  uLINK(n).gr = 0;  uLINK(n).u = 0;
end
SetupRigidBody(ID.BODY,    shapeBody, cBody);
SetupRigidBody(ID.RLEG_J5, shapeFoot, uLINK(ID.RLEG_J5).c);
SetupRigidBody(ID.LLEG_J5, shapeFoot, uLINK(ID.LLEG_J5).c);

M = TotalMass(1);                           % <-- ligne a part

% Posture non singuliere
for L = {'RLEG','LLEG'}
    uLINK(ID.([L{1} '_J2'])).q = -10*ToRad;
    uLINK(ID.([L{1} '_J3'])).q =  20*ToRad;
    uLINK(ID.([L{1} '_J4'])).q = -10*ToRad;
end

%%%%%% Conditions initiales (echelle Darwin) %%%%%
% hanche (0.1222 m sous l'origine du BODY) a 15 cm au-dessus de la cheville
% d'appui ; thigh+shank = 18.6 cm
zBody = ankleH + 0.15 + 0.1222;
uLINK(ID.BODY).p = [0 0 zBody]';
uLINK(ID.BODY).R = eye(3);
uLINK(ID.BODY).v = [0 0 0]';
uLINK(ID.BODY).w = [0 0 0]';
ForwardKinematics(1);

Rfoot.p = [ 0,     -0.074, ankleH+0.02]';   % pied droit souleve de 2 cm
Rfoot.R = eye(3);  Rfoot.v = [0 0 0]';  Rfoot.w = [0 0 0]';
Lfoot.p = [-0.015,  0.0,   ankleH]';        % pied d'appui : semelle a z = 0
Lfoot.R = eye(3);  Lfoot.v = [0 0 0]';  Lfoot.w = [0 0 0]';

InverseKinematicsAll(ID.RLEG_J5, Rfoot);
InverseKinematicsAll(ID.LLEG_J5, Lfoot);
ForwardVelocity(1);

com = calcCoM;     % Center of mass
Zc  = com(3);      % Height of the linear inverted pendulum (LIPM)
Tc  = sqrt(Zc/G);  % Time constant of LIPM
cx0 = com(1);
cy0 = com(2);

fprintf('Zc=%.3f  Tc=%.3f  ecart CoM-ZMP: x=%.4f y=%.4f m\n', ...
        Zc, Tc, cx0-Lfoot.p(1), cy0-Lfoot.p(2));

P1 = calcP(1);
L1 = calcL(1);

Dtime = 0.01;
EndTime = 0.4;
time = 0:Dtime:EndTime;
tsize = length(time);
com_m = zeros(tsize,3);
zmp_m = zeros(tsize,2);

figure
k = 1;
zmpz = 0.0;
for k = 1:tsize
    [px,vx] = LIPM(time(k),Lfoot.p(1),cx0,Tc);
    [py,vy] = LIPM(time(k),Lfoot.p(2),cy0,Tc);
    uLINK(ID.BODY).p = [px, py, zBody]';          % au lieu de 0.5
    uLINK(ID.BODY).v = [vx, vy, 0.0]';

    InverseKinematicsAll(ID.LLEG_J5, Lfoot); % Keep support foot on the ground
    ForwardVelocity(1);

    %%% Calculate ZMP
    com = calcCoM;    % Center of mass
    P   = calcP(1);   % Linear momentum
    L   = calcL(1);   % Angular momentum

    dP = (P-P1)/Dtime;
    dL = (L-L1)/Dtime;
    [zmpx,zmpy] = calcZMP(com,dP,dL,zmpz);
    P1 = P;
    L1 = L;

    com_m(k,:) = com';
    zmp_m(k,:) = [zmpx, zmpy];

    hold off
    newplot
    DrawAllJoints(1);
    h(1) = DrawMarker([zmpx,zmpy,zmpz]','r');
    h(2) = DrawMarker([com(1),com(2),0]','b');
    legend(h,'IZMP','CoM');
    axis equal
    set(gca,...
    'CameraPositionMode','manual','CameraPosition',[1,1,0.25],...
    'CameraViewAngleMode','manual','CameraViewAngle',15,...
    'Projection','perspective',...
    'XLimMode','manual','XLim',[-0.15 0.15],...
    'YLimMode','manual','YLim',[-0.15 0.15],...
    'ZLimMode','manual','ZLim',[0 0.4])
     grid on
     text(0.1, -0.1, 0.35, ['time=',num2str(time(k),'%5.3f')])
    drawnow;
end
