import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
from scipy.linalg import sqrtm
import math 

# === 1. Charger le fichier TXT ===
file_path = "data.txt"

mag_x = []
mag_y = []
mag_z = []
acc_x = []
acc_y = []
acc_z = []

with open(file_path, "r") as f:
    lines = f.readlines()

# Ignorer la première ligne (en-tête)
for line in lines[1:]:
    parts = line.strip().split()
    if len(parts) == 6:
        mag_x.append(float(parts[0]))
        mag_y.append(float(parts[1]))
        mag_z.append(float(parts[2]))
        acc_x.append(float(parts[3]))
        acc_y.append(float(parts[4]))
        acc_z.append(float(parts[5]))

# === 2. Créer la figure 3D ===
fig = plt.figure(figsize=(8, 6))
ax = fig.add_subplot(111, projection='3d')

# === 3. Afficher un nuage de points coloré par l’ordre des mesures ===
sc = ax.scatter(mag_x, mag_y, mag_z, c=range(len(mag_x)), cmap='viridis')

# === 4. Ajouter labels, titre et barre de couleur ===
ax.set_xlabel("Mag X")
ax.set_ylabel("Mag Y")
ax.set_zlabel("Mag Z")
ax.set_title("Nuage de points 3D du champ magnétique mesuré")
fig.colorbar(sc, ax=ax, label="Indice de mesure (≈ temps)")

plt.show()

M=[]
for i in range(len(mag_x)):
    x1,x2,x3=mag_x[i], mag_y[i], mag_z[i]
    Xi=[x1**2,x2**2,x3**2,x1*x2,x1*x3,x2*x3,x1,x2,x3]
    M.append(Xi)
M=np.array(M)

I=np.array([1 for _ in range(len(mag_x))])

p=np.linalg.inv(M.T@M)@M.T@I

p1,p2,p3,p4,p5,p6,p7,p8,p9=p
print(p)
Q=np.array([[p1,p4/2,p5/2],
            [p4/2,p2,p6/2],
            [p5/2,p6/2,p3]])

Q_sqrt=sqrtm(Q)

b=-1/2 * Q_sqrt@[p6,p7,p9]

y = []
for i in range(len(mag_x)):
    y.append(Q_sqrt@(np.array([mag_x[i],mag_y[i],mag_z[i]])-b))

y=np.array(y)


fig1 = plt.figure()
ax = fig1.add_subplot(111, projection='3d')

ax.scatter(mag_x, mag_y, mag_z, c='b', label='X')

ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')
ax.legend()
plt.show()

fig2 = plt.figure()
ax = fig2.add_subplot(111, projection='3d')

ax.scatter(y[:,0], y[:,1], y[:,2], c='r', label='Y')

ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')
ax.legend()
plt.show()


n=40
theta=[2*np.pi*i/n for i in range(n)]
phi=[-np.pi/2+np.pi*i/n for i in range(n)]

X_S2=[]
for i in range(n):
    for j in range(n):
        x = np.cos(theta[i]) * np.cos(phi[j])
        y0 = np.sin(theta[i]) * np.cos(phi[j])
        z = np.sin(phi[j])

        Y_S2 = np.array([x, y0, z])
        X_S2.append(np.linalg.inv(Q_sqrt) @ Y_S2 + b)

X_S2=np.array(X_S2)
mag=np.array([mag_x,mag_y,mag_z]).T

fig3 = plt.figure()
ax = fig3.add_subplot(111, projection='3d')

ax.scatter(X_S2[:,0],X_S2[:,1], X_S2[:,2], c='r', label='X_S2')
ax.scatter(mag[:,0], mag[:,1], mag[:,2], c='b', label='mag')

ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')
ax.legend()
plt.show()

acc=np.array([acc_x,acc_y,acc_z]).T
a1=np.array([0,0,1])

'''R_matrices=[]
for i in range (len(y)):
    C1 = (y[i]-((y[i]).T@acc[i])*acc[i])/np.linalg.norm(y[i]-(y[i].T@acc[i])*acc[i])
    C2 = np.cross(acc[i],C1)
    C3=acc[i]
    R=np.array([C1,C2,C3]).T
    R_matrices.append(R)
R_matrices=np.array(R_matrices)'''

R_matrices=[]
for i in range (len(y)):
    C1 = (y[i]-((y[i]).T@a1)*a1)/np.linalg.norm(y[i]-(y[i].T@a1)*a1)
    C2 = np.cross(a1,C1)
    C3=a1
    R=np.array([C1,C2,C3]).T
    R_matrices.append(R)
R_matrices=np.array(R_matrices)

phi_mat=[]
theta_mat=[]
psi_mat=[]
for i in range (len(R_matrices)):
    R=R_matrices[i]
    #phi=np.arctan2(R[2,1],R[2,2])
    #theta=np.arcsin(-R[2,0])
    print(R[1,0])
    print(R[0,0])
    print(R)
    psi=np.arctan2(R[1,0].real,R[0,0].real)
    #phi_mat.append(phi)
    #theta_mat.append(theta)
    psi_mat.append(psi)
    print(psi)
print(Q_sqrt)
print(b)

