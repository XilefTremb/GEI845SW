clc
clear all
close all

addpath(genpath('Fct kajita'));

global uLINK

% Initialisation du robot
ID = SetupDarwinRobot();

% Masse totale
M = TotalMass(ID.BODY);

% Centre de masse global
CoM = calcCoM();

% Positions des chevilles
pR = uLINK(ID.RLEG_J5).p;
pL = uLINK(ID.LLEG_J5).p;

% Affichage des resultats
disp('Masse totale [kg] :');
disp(M);

disp('Centre de masse [m] :');
disp(CoM);

disp('Position du BODY [m] :');
disp(uLINK(ID.BODY).p);

disp('Position cheville droite [m] :');
disp(pR);

disp('Position cheville gauche [m] :');
disp(pL);

% Visualisation du robot
figure(1);
DrawAllJoints(ID.BODY);
alpha(0.3);

hold on;
plot3(CoM(1), CoM(2), CoM(3),'ro', 'MarkerSize', 10, 'MarkerFaceColor', 'r');

axis equal;
grid on;

xlabel('X [m]');
ylabel('Y [m]');
zlabel('Z [m]');
title('Modele cinematique du Darwin-OP');

view(135, 20);