clc 
clear all
close all

addpath(genpath('Fct kajita'));

global uLINK

% Initialisation du robot -------------------------------------------------
ID = SetupDarwinRobot();

M = TotalMass(ID.BODY);

% Posture initiale (Half-Sitting) -----------------------------------------
theta = deg2rad(20); % angle de flexion de la hanche par rapport au body
% Permet d'éviter de commencer la sim en position de singularité

% Jambe droite
uLINK(ID.RLEG_J2).q = -theta;
uLINK(ID.RLEG_J3).q = 2*theta;
uLINK(ID.RLEG_J4).q = -theta;

% Jambe gauche
uLINK(ID.LLEG_J2).q = -theta;
uLINK(ID.LLEG_J3).q = 2*theta;
uLINK(ID.LLEG_J4).q = -theta;

% Dimensions du robot
hipOffsetZ = -uLINK(ID.RLEG_J0).b(3);
hipLength  = -uLINK(ID.RLEG_J1).b(3);
thighLength = -uLINK(ID.RLEG_J3).b(3);
tibiaLength = -uLINK(ID.RLEG_J4).b(3);
ankleH = 0.0335;

% Calcul de la hauteur du BODY
zBody = hipOffsetZ + hipLength + (thighLength + tibiaLength)*cos(theta) + ankleH;

% Position et orientation initiale du BODY en half-sitting 
uLINK(ID.BODY).p = [0; 0; zBody];
uLINK(ID.BODY).R = eye(3);

% Mise a jour de la cinematique directe
ForwardKinematics(ID.BODY);

% Centre de masse initial
CoM0 = calcCoM();

% Positions initiales des chevilles en half-sitting
pR0 = uLINK(ID.RLEG_J5).p;
pL0 = uLINK(ID.LLEG_J5).p;

% Generation des empreintes de pas ----------------------------------------
stride = 0.020;
nSteps = 6;

% Positions initiales des pieds
init.R = pR0;
init.L = pL0;
cur = init;

% Struct des pas
steps = struct('side', {}, 'p', {});

for k = 1:nSteps
    % Varier entre pied d'appui et celui qui swing
    if mod(k,2) == 1
        swing = 'R';
        support = 'L';
    else
        swing = 'L';
        support = 'R';
    end

    % Determiner la longueur du pas
    if k == 1
        stepLength = stride/2;
    elseif k == nSteps
        stepLength = 0;
    else
        stepLength = stride;
    end

    % Pied qui swing devient-> support + stepLength
    cur.(swing)(1) = cur.(support)(1) + stepLength;

    % Sauvegarder l'empreinte
    steps(k).side = swing;
    steps(k).p = cur.(swing);
end

% Generation de la trajectoire du ZMP -------------------------------------

% Parametres temporels
dt = 0.01; % pas de temps
Tinit = 1.0; % maintien position initiale
Tds = 0.1; % double support
Tss = 0.4; % simple support
Tend = 1.0; % maintien position finale

% Nombre d'echantillons par phase
Ninit = round(Tinit/dt);
Nds = round(Tds/dt);
Nss = round(Tss/dt);
Nend = round(Tend/dt);

% Position initiale du ZMP
zmp0 = CoM0(1:2);

% Positions courantes des pieds
cur = init;

% Phase initiale : ZMP immobile
zmpRef = repmat(zmp0, 1, Ninit); % copier le zmp0 Ninit fois

% Etat d'appui
% 0 = double appui
% 1 = appui droit
% 2 = appui gauche
state = zeros(1, Ninit);

% Position precedente du ZMP
zmpPrev = zmp0;

% Facteur d'interpolation du double appui
alphaDS = (0:Nds-1)/Nds;

for k = 1:nSteps

    % Determiner le pied oscillant et le pied d'appui
    swing = steps(k).side;

    if swing == 'R'
        support = 'L';
        supportState = 2;
    else
        support = 'R';
        supportState = 1;
    end

    % ZMP desire pendant le simple appui
    zmpSupport = cur.(support)(1:2);

    % Double appui : transfert du ZMP
    zmpDS = zmpPrev + (zmpSupport - zmpPrev)*alphaDS;

    zmpRef = [zmpRef, zmpDS];
    state = [state, zeros(1,Nds)];

    % Instant de decollage du pied oscillant
    steps(k).tLift = size(zmpRef,2)*dt;

    % Simple appui : ZMP fixe
    zmpSS = repmat(zmpSupport, 1, Nss);

    zmpRef = [zmpRef, zmpSS];
    state = [state, supportState*ones(1,Nss)];

    % Instant de contact du pied oscillant
    steps(k).tLand = size(zmpRef,2)*dt;

    % Mise a jour de la position du pied
    cur.(swing) = steps(k).p;

    % Mise a jour du ZMP precedent
    zmpPrev = zmpSupport;
end

% Position finale du ZMP : milieu des deux pieds
zmpEnd = (cur.R(1:2) + cur.L(1:2))/2;

% Dernier double appui
zmpDS = zmpPrev + (zmpEnd - zmpPrev)*alphaDS;

zmpRef = [zmpRef, zmpDS];
state = [state, zeros(1,Nds)];

% Maintien final
zmpRef = [zmpRef, repmat(zmpEnd,1,Nend+1)];
state = [state, zeros(1,Nend+1)];

% Vecteur temporel
N = size(zmpRef,2);
tt = (0:N-1)*dt;

Ttot = tt(end);

% Generation de la trajectoire du centre de masse -------------------------

% Parametres du LIPM
g = 9.81;
zc = CoM0(3);
Tc = sqrt(zc/g);

beta = dt^2/Tc^2;

% Positions initiale et finale desirees du CoM
comStart = CoM0(1:2);
comEnd = zmpEnd;

% Construction de la matrice A
A = zeros(N,N);

% Condition initiale
A(1,1) = 1;

% Equations du LIPM discretise
for k = 2:N-1
    A(k,k-1) = -1;
    A(k,k)   = 2 + beta;
    A(k,k+1) = -1;
end

% Condition finale
A(N,N) = 1;

% Construction des vecteurs B
Bx = beta*zmpRef(1,:)';
By = beta*zmpRef(2,:)';

% Conditions initiales
Bx(1) = comStart(1);
By(1) = comStart(2);

% Conditions finales
Bx(N) = comEnd(1);
By(N) = comEnd(2);

% Resolution du systeme selon X et Y
Xc = A \ Bx;
Yc = A \ By;

% Trajectoire 3D du centre de masse
comRef = [Xc';
          Yc';
          zc*ones(1,N)];

% Vitesses du centre de masse
comVel = zeros(3,N);

comVel(1,:) = gradient(comRef(1,:),dt);
comVel(2,:) = gradient(comRef(2,:),dt);

% Accelerations du centre de masse (equations LIPM)
comAcc = zeros(3,N);

comAcc(1,:) = (comRef(1,:) - zmpRef(1,:))/Tc^2;
comAcc(2,:) = (comRef(2,:) - zmpRef(2,:))/Tc^2;

% Generation de la trajectoire du BODY ------------------------------------

% Position initiale du BODY
body0 = uLINK(ID.BODY).p;

% Decalage initial entre le CoM et le BODY
bodyOffset = body0 - CoM0;

% Trajectoire de position du BODY
bodyRef = comRef + bodyOffset;

% Orientation constante du BODY
bodyR = eye(3);

% Vitesses et accelerations du BODY
bodyVel = comVel;
bodyAcc = comAcc;

% Generation des trajectoires des pieds -----------------------------------

% Hauteur maximale de levee du pied
stepHeight = 0.020;

% Initialisation des trajectoires
footRRef = repmat(init.R, 1, N);
footLRef = repmat(init.L, 1, N);

% Positions courantes des pieds
cur = init;

for k = 1:nSteps

    % Pied oscillant
    swing = steps(k).side;

    % Positions initiale et finale du pied
    pStart = cur.(swing);
    pEnd = steps(k).p;

    % Instants de decollage et d'atterrissage
    tLift = steps(k).tLift;
    tLand = steps(k).tLand;

    % Indices temporels
    iLift = round(tLift/dt) + 1;
    iLand = round(tLand/dt) + 1;

    % Temps normalise
    s = (0:iLand-iLift)/(iLand-iLift);

    % Interpolation cubique (X et Y)
    f = 3*s.^2 - 2*s.^3;

    % Interpolation de degre 4 (Z)
    h = 16*s.^2.*(1-s).^2;

    % Trajectoire du pied oscillant
    pSwing = pStart + (pEnd-pStart)*f;
    pSwing(3,:) = pSwing(3,:) + stepHeight*h;

    % Mise a jour de la trajectoire du pied
    if swing == 'R'
        footRRef(:,iLift:iLand) = pSwing;
        footRRef(:,iLand:end) = ...
            repmat(pEnd,1,N-iLand+1);
    else
        footLRef(:,iLift:iLand) = pSwing;
        footLRef(:,iLand:end) = ...
            repmat(pEnd,1,N-iLand+1);
    end

    % Mise a jour de la position courante
    cur.(swing) = pEnd;
end

% Orientation constante des pieds
footRRot = eye(3);
footLRot = eye(3);

% Generation des trajectoires articulaires --------------------------------

% Parametres de convergence
lambdaCoM = 0.8;
tolCoM = 1e-5;
tolIK = 1e-5;
maxIterCoM = 30;

% Sauvegarde de la posture initiale
uLINK_initial = uLINK;

% Indices des articulations
idxR = FindRoute(ID.RLEG_J5);
idxL = FindRoute(ID.LLEG_J5);

% Premiere estimation de la trajectoire du BODY
bodyEstimate = bodyRef;

% Initialisation des trajectoires finales
bodyRef = zeros(3,N);
comActual = zeros(3,N);

qR = zeros(6,N);
qL = zeros(6,N);

ikErrR = zeros(1,N);
ikErrL = zeros(1,N);

iterCoM = zeros(1,N);
convergedCoM = false(1,N);

for k = 1:N
    % Estimation initiale du BODY
    if k == 1
        bodyPos = bodyEstimate(:,k);
    else
        previousCorrection = bodyRef(:,k-1) - bodyEstimate(:,k-1);
        bodyPos = bodyEstimate(:,k) + previousCorrection;
    end

    % Poses desirees des pieds
    TargetR.p = footRRef(:,k);
    TargetR.R = footRRot;

    TargetL.p = footLRef(:,k);
    TargetL.R = footLRot;

    % Correction iterative du centre de masse
    for i = 1:maxIterCoM
        % Pose du BODY
        uLINK(ID.BODY).p = bodyPos;
        uLINK(ID.BODY).R = bodyR;

        % Cinematique inverse
        errR = InverseKinematics(ID.RLEG_J5,TargetR);
        errL = InverseKinematics(ID.LLEG_J5,TargetL);

        % Cinematique directe et centre de masse
        ForwardKinematics(ID.BODY);
        CoM = calcCoM();

        % Erreur du centre de masse
        eCoM = comRef(:,k) - CoM;

        % Verification de la convergence
        if norm(eCoM) < tolCoM && errR < tolIK && errL < tolIK
            convergedCoM(k) = true;
            break;
        end

        % Correction de la position du BODY
        if i < maxIterCoM
            bodyPos = bodyPos + lambdaCoM*eCoM;
        end
    end

    % Verification de la convergence
    if ~convergedCoM(k)
        uLINK = uLINK_initial;

        error('Correction non convergente a t = %.3f s',tt(k));
    end

    % Sauvegarde des trajectoires
    bodyRef(:,k) = uLINK(ID.BODY).p;
    comActual(:,k) = CoM;

    qR(:,k) = [uLINK(idxR).q]';
    qL(:,k) = [uLINK(idxL).q]';

    ikErrR(k) = errR;
    ikErrL(k) = errL;

    iterCoM(k) = i;
end

% Erreur residuelle du centre de masse
comError = comRef - comActual;

% Restauration de la posture initiale
uLINK = uLINK_initial;

% Calcul des derivees temporelles -----------------------------------------

% Derivees articulaires
dqR = zeros(6,N);
dqL = zeros(6,N);

ddqR = zeros(6,N);
ddqL = zeros(6,N);

for j = 1:6
    dqR(j,:) = gradient(qR(j,:),dt);
    dqL(j,:) = gradient(qL(j,:),dt);

    ddqR(j,:) = gradient(dqR(j,:),dt);
    ddqL(j,:) = gradient(dqL(j,:),dt);
end

% Derivees du BODY
bodyVel = zeros(3,N);
bodyAcc = zeros(3,N);

for j = 1:3
    bodyVel(j,:) = gradient(bodyRef(j,:),dt);
    bodyAcc(j,:) = gradient(bodyVel(j,:),dt);
end

% Exportation des trajectoires articulaires -------------------------------

% Conversion des conventions Kajita vers URDF
sR = [-1 -1 +1 +1 -1 +1]';
sL = [-1 -1 -1 -1 +1 +1]';

qR_urdf = sR .* qR;
qL_urdf = sL .* qL;

% Matrice des donnees a exporter
exportData = [tt; qR_urdf; qL_urdf]';

% Nom du fichier
fileName = 'dynamic_test.txt';

% Ouverture du fichier
fid = fopen(fileName,'w');

% Nombre d'echantillons
fprintf(fid,'%d\n',N);

% Periode d'echantillonnage
fprintf(fid,'%.10e\n',dt);

% Format : temps + 12 angles articulaires
lineFormat = [repmat('%.10e\t',1,12),'%.10e\n'];

% Ecriture des trajectoires
fprintf(fid,lineFormat,exportData');

% Fermeture du fichier
fclose(fid);

fprintf('Fichier genere : %s\n',fileName);
fprintf('Nombre echantillons : %d\n',N);
fprintf('Duree de la marche : %.2f s\n',Ttot);