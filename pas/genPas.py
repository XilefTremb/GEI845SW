import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits import mplot3d

def cubicTrajectory(Pe, Pd, Ts):
    a0 = Pe
    a3 = 2*(Pe-Pd)/(Ts[-1]**3)
    a2 = -3*a3*Ts[-1]/2 
    
    return a3*(Ts**3)+a2*(Ts**2)+a0

def stepTrajectory(Pe, ymax, Ts):
    
    a0 = Pe
    a1 = 0
    a4 = (16*ymax)/(Ts[-1]**4)
    a3 = -2*a4*Ts[-1]
    a2 = -1/2*a4*(Ts[-1]**2)-3/4*a3*Ts[-1]
    
    return a4*(Ts**4)+a3*(Ts**3)+a2*(Ts**2)+a1*Ts+a0


def createT(a, alpha, d, theta):
    n1 = np.cos(theta)
    n2 = -np.sin(theta)*np.cos(alpha)
    n3 = np.sin(theta)*np.sin(alpha)
    n4 = a*np.cos(theta)
    n5 = np.sin(theta)
    n6 = np.cos(theta)*np.cos(alpha)
    n7 = -np.cos(theta)*np.sin(alpha)
    n8 = a*np.sin(theta)
    n9 = np.sin(alpha)
    n10 = np.cos(alpha)
    n11 = d
    return np.array([[n1,n2,n3,n4],[n5,n6,n7,n8],[0,n9,n10,n11],[0,0,0,1]])


def BuildDH(Q):
    L1 = 0.10123734488813899
    L2 = 0.0335
    DH = [[0,   np.pi/2,    0, Q[0][0]],
          [0,   np.pi/2,   0, Q[1][0]+np.pi/2],
          [L1,  0,          0, Q[2][0]],
          [L1,  np.pi,      0, Q[3][0]],
          [0,   -np.pi/2,   0, Q[4][0]],
          [L2,  0,          0, Q[5][0]]]
    return DH


def calcJacobienne(matDict):
    z0 = np.array([[0],[0],[1]])
    pe = matDict[str(len(matDict)-1)][0:3, [3]]
    Jp = np.cross(z0, pe, axis=0)
    matJacob = np.vstack((Jp,z0))
    
    for i in range(2, len(matDict)):
        Jp = np.cross(matDict[str(i-1)][0:3, [2]], matDict[str(len(matDict)-1)][0:3, [3]] - matDict[str(i-1)][0:3, [3]], axis=0)
        Jt = matDict[str(i-1)][0:3, [2]]
        jacobLien = np.vstack((Jp,Jt))
        matJacob = np.hstack((matJacob, jacobLien))

    return matJacob


def dampedLeastSquares(J, lamb):
    a = np.matmul(J,J.T)
    size = np.shape(a)
    b = lamb*np.identity(size[0])
    c = np.linalg.inv(a+b)
    result = np.matmul(J.T, c)
    return result

def calcMatS(x):
    return np.array([[0,      -x[2,0], x[1,0]],
                     [x[2,0],  0,      x[0,0]],
                     [-x[1,0], x[0,0], 0]])

def calcL(Re, Rd):
    xe = Re[0:3,[0]]
    ye = Re[0:3,[1]]
    ze = Re[0:3,[2]]
    xd = Rd[0:3,[0]]
    yd = Rd[0:3,[1]]
    zd = Rd[0:3,[2]]
    L = -0.5*(np.matmul(calcMatS(xd),calcMatS(xe))+np.matmul(calcMatS(yd),calcMatS(ye))+np.matmul(calcMatS(zd),calcMatS(ze)))
    return L 

def calcErreurAngle(Re, Rd):
    #e0_chap = l⁻¹*e0
    #e0 = 0.5*(xe x xd + ye x yd + ze x zd)
    xe = Re[0:3,[0]]
    ye = Re[0:3,[1]]
    ze = Re[0:3,[2]]
    xd = Rd[0:3,[0]]
    yd = Rd[0:3,[1]]
    zd = Rd[0:3,[2]]
    e0 = 0.5*(np.cross(xe, xd, axis=0)+np.cross(ye, yd, axis=0)+np.linalg.cross(ze, zd, axis=0))
    erreur = np.matmul(dampedLeastSquares(calcL(Re, Rd), 10**-3),e0)
    return erreur

def invertHomogen(mat):
    R = np.array(mat[0:3, 0:3]).T
    P = np.matmul(-R,mat[0:3,[3]])
    return np.vstack((np.hstack((R,P)), [[0,0,0,1]]))

def fromEEto0(Q, dest):
    mats = FK(Q)
    #Eto0 = invertHomogen(mats[str(len(mats)-1)])
    return np.matmul(mats[str(len(mats)-1)], dest)

def from0toPelvis(Q, pelvis):
    return 0

def FK(Q):
    DH = BuildDH(Q)
    homo = np.array([[1,0,0,0],[0,1,0,0],[0,0,1,0],[0,0,0,1]])
    matDict = {}
    matDict["0"] = homo
    linkN = 1
    for link in DH:      
        homo = np.matmul(homo,createT(link[0], link[1], link[2], link[3]))
        matDict[str(linkN)] = homo      
        linkN += 1        
    return matDict

def rotMatToAngle(mat):
    rx = np.atan2(mat[2,1],mat[2,2])
    ry = np.atan2(-mat[2,0], np.sqrt(mat[2,1]**2+mat[2,2]**2))
    rz = np.atan2(mat[1,0],mat[0,0])
    return np.array([[rx],[ry],[rz]])

#TODO mostly done, reste à confirmer avec mes notes pour l'erreur d'angle et R1, R2
def IK(Q, matd, DeltaT, Eps1, Eps2, imax):
    Pd = matd[0:3,[3]]
    Thetad = matd[0:3,0:3]
    matsDict = FK(Q)
    Pe = matsDict[str(len(matsDict)-1)][0:3,[3]]
    Thetae = matsDict[str(len(matsDict)-1)][0:3,0:3]
    Er1 = np.linalg.norm(Pd - Pe)
    Er2 = np.linalg.norm(calcErreurAngle(Thetae, Thetad)) # revoir l'erreur d'orientation
    K = 0 #compteur anti blocage
    while (Er1 > Eps1 and K < imax) or (Er2 >Eps2 and K< imax*3/4):
        J = calcJacobienne(matsDict)
        J1 = J[0:3, :]
        J2 = J[3:6, :]
        J1_inv = dampedLeastSquares(J1, 10**-3)
        J2_chap_inv = dampedLeastSquares(np.matmul(J2,(np.identity(J1.shape[1])-np.matmul(J1_inv,J1))), 10**-3)
        R1point = (Pd-Pe)/DeltaT
        #R2point = (rotMatToAngle(Thetad)-rotMatToAngle(Thetae))/DeltaT
        R2point = calcErreurAngle(Thetae, Thetad)
        Qpoint = np.matmul(J1_inv, R1point) + np.matmul(J2_chap_inv, (R2point - np.matmul(np.matmul(J2,J1_inv),R1point))) #J1_inv * R1 + J2_chap_inv(R2 - J2*J1_inv*R1)
        #Qpoint = np.matmul(J1_inv, R1point)
        Q = Qpoint*DeltaT + Q
        matsDict = FK(Q)
        Pe = matsDict[str(len(matsDict)-1)][0:3,[3]]
        Thetae = matsDict[str(len(matsDict)-1)][0:3,0:3]
        Er1 = np.linalg.norm(Pd - Pe)
        Er2 = np.linalg.norm(calcErreurAngle(Thetae, Thetad)) 
        K+=1
    logQ(Q, DeltaT)
    return Q


def displayJoints3d(MatsArr, TrajArr, repere):
    fig = plt.figure()
    ax = plt.axes(projection='3d')
    k=0


    for m in MatsArr:
        m["0"] = np.matmul(repere, m["0"])
        x =[m["0"][0][3]]
        y =[m["0"][1][3]]
        z =[m["0"][2][3]]
        for i in range(1, len(m)):
            m[str(i)] = np.matmul(repere, str(i))
            x.append(m[str(i)][0][3])
            y.append(m[str(i)][1][3])
            z.append(m[str(i)][2][3])
        ax.plot3D(x,y,z, marker='o', label=str(k))
        ax.scatter3D(x[0], y[0], z[0], color='green', s=25, zorder=2)
        ax.scatter3D(x[-1], y[-1], z[-1], color='red', s=25, zorder=2)
        k+=1
    
    if TrajArr != []:
        x =[]
        y =[]
        z =[]
        for p in TrajArr:
            p = np.matmul(repere,p)
            x.append(p[0][3])
            y.append(p[1][3])
            z.append(p[2][3])
        ax.plot3D(x,y,z, marker='x', label="trajectoire", color='black')
    
    ax.legend()
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    #ax.set_xlim(-0.1,0.3)
    #ax.set_ylim(-0.1,0.3)
    #ax.set_zlim(-0.01,0.3)

    ax.grid()
    plt.show()

#génère la trajectoire du pas dans le domaine du pied
#from DH, en regardan le robot de derrière
#   X vers le sol -> traj 4e deg
#   Y vers la gauche -> traj cubique
#   Z vers l'avant -> traj cubique
#   deltaT est le temps du mouvement complet voulu
def genereTrajectoirePas(devant, gauche, HauteurPas, nPoints):
    Ts = np.linspace(0,1,nPoints)
    X = -stepTrajectory(0,HauteurPas, Ts)
    Y = cubicTrajectory(0,gauche, Ts)
    Z = cubicTrajectory(0,devant, Ts)
    if gauche!=0:
        theta = np.arctan(gauche/devant)
    else:
        theta = 0
    
    #return Ts for graphing purpose
    return np.array([X,Y,Z,np.full_like(X,theta)]), Ts

#add angle and time 
def logQ(Q, dT):
    global Qout
    global Tout
    Qout.append(Q)
    #for first time
    if len(Tout)==0:
        Tout.append(dT)
    else:    
        Tout.append(Tout[-1]+dT)

def writeCommandText(rate, filename):
    #npoint
    #rate
    #t j1 j2 j3 j4 j5 j6   (repeat for every point)
    global Qout
    global Tout
    f = open(filename, "w+")
    f.write(str(len(Qout))+"\n")
    f.write("%.6e"%rate+"\n")
    for i in range(len(Qout)):        
        f.write("%.6e"%Tout[i]+"\t%.6e"%Qout[i][0,0]+"\t%.6e"%(Qout[i][1,0])+"\t%.6e"%Qout[i][2,0]+"\t%.6e"%Qout[i][3,0]+"\t%.6e"%Qout[i][4,0]+"\t%.6e"%Qout[i][5,0]+"\n")
    f.close()
    

global Qout
global Tout
Qout = []
Tout = []

Q0 = [[0],[0.3],[-0.4],[0.81],[0.4],[0.3]]
#5 cm vers l'avant, 2cm vers la droite avec une hauteur de pas de 3cm, la trajectoire est séparé en 25 points
Trajectoire, _ = genereTrajectoirePas(0.06, -0.01, 0.03, 25)
GoalMats = []
Q=Q0 #Q0 est la position de départ, Q est la position qui avance
chartArr = [FK(Q)]
#displayJoints3d(chartArr,[])
rate = 0.01
goalEE = np.array([[1, 0, 0, 0],
                   [0, 1, 0, 0],
                   [0, 0, 1, 0],
                   [0, 0, 0, 1]])
Z0Goal = fromEEto0(Q0,goalEE)
chartArr = [FK(Q)]
chartTraj = [Z0Goal]
for i in range(1, len(Trajectoire[0,:])):
    point = Trajectoire[:,i]
    #goal avec angle de jambe
    #goalEE = np.array([ [1, 0,                 0,                point[0]],
    #                    [0, np.cos(point[3]), -np.sin(point[3]), point[1]],
    #                    [0, np.sin(point[3]),  np.cos(point[3]), point[2]],
    #                    [0, 0,                 0,                1]])
    #
    goalEE = np.array([[1, 0, 0, point[0]],
                       [0, 1, 0, point[1]],
                       [0, 0, 1, point[2]],
                       [0, 0, 0, 1]])
    Z0Goal = fromEEto0(Q0,goalEE)
    
    Q = IK(Q, Z0Goal, rate, 0.001, 0.01, 1)
    chartArr.append(FK(Q))
    chartTraj.append(Z0Goal)
repere = np.array([[1, 0, 0, 0],[0, 1, 0, 0],[0, 0, 1, 0],[0, 0, 0, 1]]) #identité pour aucun changement de repère
displayJoints3d(chartArr,chartTraj,repere)
writeCommandText(rate, "text.txt")
    


