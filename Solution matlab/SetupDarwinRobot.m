%%% SetupDarwinRobot.m  (FONCTION : ID = SetupDarwinRobot())
%%% Version Darwin-OP de SetupBipedRobot.m (Kajita) : meme structure, memes noms
%%% de liens (BODY, RLEG_J0..J5, LLEG_J0..J5) et meme ordre (jambe droite
%%% = liens 2-7, jambe gauche = liens 8-13) pour que le code du livre
%%% (IK_leg, calcMC, etc.) fonctionne sans modification.
%
% Correspondance : J0 = MP_PELVIS (lacet hanche), J1 = MP_THIGH1 (roulis hanche),
% J2 = MP_THIGH2 (tangage hanche), J3 = MP_TIBIA (genou),
% J4 = MP_ANKLE1 (tangage cheville), J5 = MP_ANKLE2 (roulis cheville)
%
% b, a, c sont exprimes dans le repere commun du body (q = 0), pas dans les
% reperes locaux de l'URDF. Axes de la convention du livre (UZ UX UY UY UY UX).
% Signes URDF (q_urdf = s .* q_kajita) :
%   jambe droite : s = [-1 -1 +1 +1 -1 +1]
%   jambe gauche : s = [-1 -1 -1 -1 +1 +1]

function ID = SetupDarwinRobot()

global uLINK

ToDeg = 180/pi;
ToRad = pi/180;
UX = [1 0 0]';
UY = [0 1 0]';
UZ = [0 0 1]';

% Masse du tronc = body + cou + tete + 2 bras (URDF)
mArm    = 0.010 + 0.025913024 + 0.16837715 + 0.059288504 + 3*0.010;
mBody   = 0.97559947 + 0.024357719 + 0.15804192 + 2*mArm;     % 1.745156465 kg
cBody   = [-0.000235 -0.021006 -0.00913]';  % CoM composite du tronc (URDF); y = -2.1 cm hérité de l'URDF
mPelvisR = 0.197; % Masse du pelvis droit selon URDF
mPelvisL = 0.167; % Masse du pelvis gauche selon URDF
ankleH  = 0.0335;     % hauteur cheville -> semelle [m] : A CONFIRMER dans la doc ROBOTIS

uLINK    = struct('name','BODY'    ,'m',mBody       ,'sister',0 ,'child',2 ,'b',[0 0 0]'                 ,'a',UZ,'q',0,'c',cBody);

% Jambe droite
uLINK(2) = struct('name','RLEG_J0' ,'m',mPelvisR     ,'sister',8 ,'child',3 ,'b',[-0.005 -0.037 -0.09355]','a',UZ,'q',0,'c',[-0.00048   0        -0.018437]');
uLINK(3) = struct('name','RLEG_J1' ,'m',0.11904336  ,'sister',0 ,'child',4 ,'b',[0 0 -0.028652]'         ,'a',UX,'q',0,'c',[-0.013873 -0.000079 -0.018242]');
uLINK(4) = struct('name','RLEG_J2' ,'m',0.11904336  ,'sister',0 ,'child',5 ,'b',[0 0  0]'                ,'a',UY,'q',0,'c',[ 0.000323  0.000692 -0.062965]');
uLINK(5) = struct('name','RLEG_J3' ,'m',0.070309794 ,'sister',0 ,'child',6 ,'b',[0 0 -0.093]'            ,'a',UY,'q',0,'c',[-0.006548 -0.000592 -0.053955]');
uLINK(6) = struct('name','RLEG_J4' ,'m',0.16710792  ,'sister',0 ,'child',7 ,'b',[0 0 -0.093]'            ,'a',UY,'q',0,'c',[-0.018536  0.013873  0.000214]');  % signe de y comme dans l'URDF (non symetrique)
uLINK(7) = struct('name','RLEG_J5' ,'m',0.0794462   ,'sister',0 ,'child',0 ,'b',[0 0  0]'                ,'a',UX,'q',0,'c',[-0.000503 -0.009506 -0.025995]');

% Jambe gauche
uLINK(8) = struct('name','LLEG_J0' ,'m',mPelvisL     ,'sister',0 ,'child',9 ,'b',[-0.005  0.037 -0.09355]','a',UZ,'q',0,'c',[ 0.00048   0        -0.018437]');
uLINK(9) = struct('name','LLEG_J1' ,'m',0.11904336  ,'sister',0 ,'child',10,'b',[0 0 -0.028652]'         ,'a',UX,'q',0,'c',[-0.013873  0.000079 -0.018242]');
uLINK(10)= struct('name','LLEG_J2' ,'m',0.11904336  ,'sister',0 ,'child',11,'b',[0 0  0]'                ,'a',UY,'q',0,'c',[ 0.000323 -0.000692 -0.062965]');
uLINK(11)= struct('name','LLEG_J3' ,'m',0.070309794 ,'sister',0 ,'child',12,'b',[0 0 -0.093]'            ,'a',UY,'q',0,'c',[-0.006548  0.000592 -0.053955]');
uLINK(12)= struct('name','LLEG_J4' ,'m',0.16710792  ,'sister',0 ,'child',13,'b',[0 0 -0.093]'            ,'a',UY,'q',0,'c',[-0.018536  0.013873 -0.000214]');
uLINK(13)= struct('name','LLEG_J5' ,'m',0.0794462   ,'sister',0 ,'child',0 ,'b',[0 0  0]'                ,'a',UX,'q',0,'c',[-0.000503  0.009506 -0.025995]');

% Visualisation du robot --------------------------------------------------

% Dimensions approximatives du haut du corps et pieds
bodyDim = [0.178 0.296 0.156];
footDim = [0.104 0.066 0.008];

% Geometrie du haut du corps
[uLINK(1).vertex, uLINK(1).face] = MakeBox(bodyDim, bodyDim/2);

% Geometrie du pied droit
[uLINK(7).vertex, uLINK(7).face] = MakeBox(footDim, [footDim(1)/2 footDim(2)/2 ankleH]);

% Geometrie du pied gauche
[uLINK(13).vertex, uLINK(13).face] = MakeBox(footDim, [footDim(1)/2 footDim(2)/2 ankleH]);

% -------------------------------------------------------------------------

FindMother(1);   % Find mother link from sister and child data

%%% Indices des liens, retournes dans la structure ID (ex. ID.BODY = 1, ID.RLEG_J5 = 7)
for n=1:length(uLINK)
    ID.(uLINK(n).name) = n;
end
BODY = ID.BODY;

uLINK(BODY).p = [0.0, 0.0, 0.09355+0.028652+0.093+0.093+ankleH]';   % jambes tendues, pieds au sol
uLINK(BODY).R = eye(3);
ForwardKinematics(1);

uLINK(BODY).v = [0 0 0]';
uLINK(BODY).w = [0 0 0]';
for n=1:length(uLINK)
    uLINK(n).dq     = 0;            % joint speed   [rad/s]
end

end