%%% main.m  (etapes 2 et 3)
%%% Etape 2 : empreintes des pieds | Etape 3 : trajectoire de reference du ZMP
%%% Prerequis : le depot de Kajita (ForwardKinematics, FindMother, ...) est dans le path
%%%   addpath(genpath('chemin/vers/IntroductionToHumanoidRobotics'));

clear; clc; close all;
addpath(genpath('Fct kajita'));

%% 1) Creer le robot (remplit la variable GLOBALE uLINK, retourne les indices)
global uLINK
ID = SetupDarwinRobot();

%% 2) Lire des dimensions directement dans le modele
hipY   = uLINK(ID.LLEG_J0).b(2);          % demi-ecart des hanches (0.037 m)
legLen = -(uLINK(ID.LLEG_J1).b(3) + uLINK(ID.LLEG_J3).b(3) + uLINK(ID.LLEG_J4).b(3));  % hanche -> cheville
fprintf('Demi-ecart hanches = %.4f m, hauteur hanche (J0) -> cheville = %.4f m\n', hipY, legLen);

% Verification : position des chevilles dans la pose initiale (FK deja appelee dans le setup)
pR = uLINK(ID.RLEG_J5).p;
pL = uLINK(ID.LLEG_J5).p;
fprintf('Cheville D : [%.4f %.4f %.4f]\n', pR);
fprintf('Cheville G : [%.4f %.4f %.4f]\n', pL);

%% 3) Dimensions du pied (A CONFIRMER dans la documentation ROBOTIS)
footL  = 0.104;    % longueur du pied  [m]  (direction x)
footW  = 0.066;    % largeur du pied   [m]  (direction y)
footOx = 0.0;      % decalage x du centre de la semelle par rapport a la cheville [m]
ankleH = 0.0335;   % hauteur cheville -> semelle [m] (doit etre la meme valeur que dans SetupDarwinRobot)

fprintf('Hauteur de la semelle en pose initiale : %.4f m (doit etre ~0)\n', pR(3) - ankleH);

%% 4) Parametres de marche
stride  = 0.04;                 % longueur de pas [m]
stepW   = 2*hipY + 0.02;        % ecart lateral entre les chevilles [m] (on elargit un peu : 2*hipY ne laisse que 8 mm entre les pieds)
nSteps  = 6;                    % nombre de pas
firstSwing = 'R';               % pied qui avance en premier ('R' ou 'L')

%% 5) Generer la sequence d'empreintes (position de la CHEVILLE au sol, yaw = 0)
cur.R = [0; -stepW/2];          % pose initiale : pieds cote a cote
cur.L = [0;  stepW/2];
init = cur;

if firstSwing == 'R', sides = {'R','L'}; else, sides = {'L','R'}; end

steps = struct('side', {}, 'p', {});
for k = 1:nSteps
    s = sides{mod(k-1,2)+1};    % pied qui bouge
    o = sides{mod(k,2)+1};      % pied d'appui
    if k == 1
        x = cur.(o)(1) + stride/2;       % premier pas : demi-pas
    elseif k == nSteps
        x = cur.(o)(1);                  % dernier pas : on ramene le pied a cote de l'autre
    else
        x = cur.(o)(1) + stride;         % pas normal : on depasse le pied d'appui d'un pas
    end
    cur.(s)(1) = x;
    steps(k).side = s;
    steps(k).p    = cur.(s);             % [x; y]
end

% Affichage
for k = 1:nSteps
    fprintf('Pas %d (%s) : x = %.3f, y = %.3f\n', k, steps(k).side, steps(k).p(1), steps(k).p(2));
end

%% 6) Visualisation des empreintes (rectangles centres sur la semelle)
figure; hold on; axis equal; grid on;
xlabel('x [m]'); ylabel('y [m]'); title('Empreintes des pieds du Darwin-OP');
drawFoot = @(p, col) patch( p(1)+footOx+[-1 1 1 -1]*footL/2, p(2)+[-1 -1 1 1]*footW/2, col, 'FaceAlpha', 0.4);

drawFoot(init.R, [0.6 0.6 0.6]);  drawFoot(init.L, [0.6 0.6 0.6]);   % pose initiale (gris)
for k = 1:nSteps
    if steps(k).side == 'R', col = 'r'; else, col = 'b'; end
    drawFoot(steps(k).p, col);
    text(steps(k).p(1), steps(k).p(2), num2str(k), 'HorizontalAlignment', 'center');
end

%% 7) Sauvegarder pour l'etape 3 (trajectoire du ZMP)
save('empreintes.mat', 'steps', 'init', 'stride', 'stepW', 'footL', 'footW', 'footOx', 'ankleH');


%% =====================  ETAPE 3 : TRAJECTOIRE DU ZMP  =====================

%% 8) Parametres temporels
dt    = 0.01;    % pas d'echantillonnage [s]
Tinit = 1.0;     % double appui initial : ZMP au milieu des pieds [s]
Tds   = 0.1;     % double appui a chaque pas : le ZMP passe d'un pied a l'autre [s]
Tss   = 0.4;     % simple appui : ZMP fixe sur le pied d'appui [s]
Tend  = 1.0;     % maintien final : ZMP au milieu des pieds [s]

%% 9) CoM initial et hauteur du CoM (utile pour le pendule inverse a l'etape 4)
M = 0;  MC = zeros(3,1);
for n = 1:length(uLINK)
    M  = M  + uLINK(n).m;
    MC = MC + uLINK(n).m * (uLINK(n).p + uLINK(n).R * uLINK(n).c);
end
com0 = MC / M;              % CoM global dans la pose initiale (sol en z = 0)
zc   = com0(3);             % hauteur du CoM au-dessus du sol [m]
Tc   = sqrt(zc/9.81);       % constante de temps du pendule inverse [s]
fprintf('Masse totale = %.3f kg\n', M);
fprintf('CoM initial : [%.4f %.4f %.4f] m,  zc = %.4f m,  Tc = %.3f s\n', com0, zc, Tc);

%% 10) Segments de la trajectoire du ZMP
% Chaque segment : [t0 t1 x0 y0 x1 y1 etat]   etat : 0 = double appui, 1 = appui droit, 2 = appui gauche
% Regles : - ZMP initial au milieu des deux pieds
%          - pendant le simple appui, ZMP au centre de la semelle du pied d'appui
%          - pendant le double appui, ZMP lineaire entre l'ancien et le nouveau pied d'appui
%          - a la fin, ZMP ramene au milieu des deux pieds
off  = [footOx; 0];                       % centre de la semelle par rapport a la cheville
tmp  = init;                              % positions des pieds, mises a jour pas a pas
mid0 = (init.R + init.L)/2 + off;
seg  = [];
t    = 0;
seg(end+1,:) = [t, t+Tinit, mid0', mid0', 0];   t = t + Tinit;
prev = mid0;

for k = 1:nSteps
    s = steps(k).side;                    % pied qui avance
    if s == 'R', o = 'L'; st = 2; else, o = 'R'; st = 1; end   % o = pied d'appui
    sup = tmp.(o) + off;                  % ZMP sur le pied d'appui

    seg(end+1,:) = [t, t+Tds, prev', sup', 0];  t = t + Tds;   % double appui
    steps(k).tLift = t;                                        % decollage du pied oscillant
    seg(end+1,:) = [t, t+Tss, sup', sup', st];  t = t + Tss;   % simple appui
    steps(k).tLand = t;                                        % pose du pied oscillant

    prev = sup;
    tmp.(s) = steps(k).p;                 % le pied oscillant est maintenant pose
end

midEnd = (tmp.R + tmp.L)/2 + off;
seg(end+1,:) = [t, t+Tds,  prev', midEnd', 0];  t = t + Tds;
seg(end+1,:) = [t, t+Tend, midEnd', midEnd', 0]; t = t + Tend;
Ttot = t;

%% 11) Echantillonnage de la trajectoire
N      = round(Ttot/dt);
tt     = (0:N-1) * dt;
zmpRef = zeros(2, N);
state  = zeros(1, N);
for i = 1:size(seg,1)
    idx = tt >= seg(i,1) & tt < seg(i,2);
    a   = (tt(idx) - seg(i,1)) / (seg(i,2) - seg(i,1));
    zmpRef(1,idx) = seg(i,3) + (seg(i,5) - seg(i,3)) * a;
    zmpRef(2,idx) = seg(i,4) + (seg(i,6) - seg(i,4)) * a;
    state(idx)    = seg(i,7);
end
fprintf('Duree totale = %.2f s, %d echantillons (dt = %.3f s)\n', Ttot, N, dt);
for k = 1:nSteps
    fprintf('Pas %d (%s) : decollage t = %.2f s, pose t = %.2f s\n', k, steps(k).side, steps(k).tLift, steps(k).tLand);
end

%% 12) Figures
% a) Empreintes + trajectoire du ZMP (comme la Fig. 1 de l'enonce)
figure; hold on; axis equal; grid on;
xlabel('x [m]'); ylabel('y [m]'); title('Empreintes et trajectoire du ZMP');
drawFoot(init.R, [0.6 0.6 0.6]);  drawFoot(init.L, [0.6 0.6 0.6]);
for k = 1:nSteps
    if steps(k).side == 'R', col = 'r'; else, col = 'b'; end
    drawFoot(steps(k).p, col);
    text(steps(k).p(1), steps(k).p(2), num2str(k), 'HorizontalAlignment', 'center');
end
hZ = plot(zmpRef(1,:), zmpRef(2,:), 'k-', 'LineWidth', 2);
hC = plot(com0(1), com0(2), 'gx', 'MarkerSize', 10, 'LineWidth', 2);   % CoM initial
legend([hZ hC], {'ZMP de reference', 'CoM initial'});

% b) x(t) et y(t) du ZMP
figure;
subplot(2,1,1); plot(tt, zmpRef(1,:), 'LineWidth', 1.5); grid on; ylabel('x_{zmp} [m]'); title('ZMP de reference');
subplot(2,1,2); plot(tt, zmpRef(2,:), 'LineWidth', 1.5); grid on; ylabel('y_{zmp} [m]'); xlabel('t [s]');

%% 13) Sauvegarder pour l'etape 4 (trajectoire du CoM et du pelvis)
save('zmp.mat', 'tt', 'zmpRef', 'state', 'steps', 'init', 'dt', 'zc', 'Tc', 'com0', ...
     'Tinit', 'Tds', 'Tss', 'Tend', 'stride', 'stepW', 'footL', 'footW', 'footOx', 'ankleH');