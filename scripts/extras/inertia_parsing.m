# Torso
ixx=0.00308361; ixy=1.43116e-05; ixz=-3.30211e-05; iyy=0.0028835; iyz=-2.70793e-05; izz=0.0015924;
mass = 1.04956;
com = [-0.00413, 0, 0.04342]';
rpy=[0 -0 0];
com = (rotZ(rpy(3))*rotY(rpy(2))*rotX(rpy(1)))'*com;
I = [ixx, ixy, ixz; ixy, iyy, iyz; ixz, iyz, izz];
I = I + mass * com' * com * eye(3) + com * com';
Torso = (rotZ(rpy(3))*rotY(rpy(2))*rotX(rpy(1)))'*I*(rotZ(rpy(3))*rotY(rpy(2))*rotX(rpy(1)))

# HeadYaw
ixx=1.60324e-05; ixy=1.57e-09; ixz=3.16276e-09; iyy=1.70394e-05; iyz=-5.295e-08; izz=5.53372e-06;
mass = 0.07842;
com = [-1e-05 0 -0.02742]';
rpy=[0 -0 0];
com = (rotZ(rpy(3))*rotY(rpy(2))*rotX(rpy(1)))'*com;
I = [ixx, ixy, ixz; ixy, iyy, iyz; ixz, iyz, izz];
I = I + mass * com' * com * eye(3) + com * com';
HeadYaw = (rotZ(rpy(3))*rotY(rpy(2))*rotX(rpy(1)))'*I*(rotZ(rpy(3))*rotY(rpy(2))*rotX(rpy(1)))

# HeadPitch
ixx=0.00077097; ixy=4.76784e-06; ixz=9.62477e-06; iyy=0.000680374; iyz=-1.85791e-05; izz=0.000714702;
mass = 0.65973;
com = [0.0571855 0.00108886 0.00146278]';
rpy=[1.5708 5.55112e-17 1.5708];
#com = (rotZ(rpy(3))*rotY(rpy(2))*rotX(rpy(1)))'*com;
I = [ixx, ixy, ixz; ixy, iyy, iyz; ixz, iyz, izz];
I = I + mass * com' * com * eye(3) + com * com';
HeadPitch = (rotZ(rpy(3))*rotY(rpy(2))*rotX(rpy(1)))'*I*(rotZ(rpy(3))*rotY(rpy(2))*rotX(rpy(1)))

# LShoulderPitch
ixx=1.83025e-05; ixy=2.06011e-06; ixz=1.88776e-09; iyy=1.39005e-05; iyz=-3.66592e-07; izz=2.01862e-05;
mass = 0.09304;
com = [-0.00165 -0.000139999 -0.02663]';
rpy=[1.5708 -0 0];
#com = (rotZ(rpy(3))*rotY(rpy(2))*rotX(rpy(1)))'*com;
I = [ixx, ixy, ixz; ixy, iyy, iyz; ixz, iyz, izz];
I = I + mass * com' * com * eye(3) + com * com';
I = (rotZ(rpy(3))*rotY(rpy(2))*rotX(rpy(1)))'*I*(rotZ(rpy(3))*rotY(rpy(2))*rotX(rpy(1)));
LShoulderPitch = I

# LShoulderRoll
ixx=8.7181e-05; ixy=-2.53381e-05; ixz=-1.4213e-05; iyy=0.000274712; iyz=4.71439e-07; izz=0.000241812;
mass = 0.15777;
com = [0.00563 -0.02455 0.0033]';
rpy=[0 -0 -1.5708];
#com = (rotZ(rpy(3))*rotY(rpy(2))*rotX(rpy(1)))'*com;
I = [ixx, ixy, ixz; ixy, iyy, iyz; ixz, iyz, izz];
I = I + mass * com' * com * eye(3) + com * com';
I = (rotZ(rpy(3))*rotY(rpy(2))*rotX(rpy(1)))'*I*(rotZ(rpy(3))*rotY(rpy(2))*rotX(rpy(1)));
LShoulderRoll = I

# LElbowYaw
ixx=5.59588e-06; ixy=4.21e-09; ixz=2.92241e-07; iyy=2.66179e-05; iyz=-1.84e-09; izz=2.76294e-05;
mass = 0.06483;
com = [1.19332e-09 -0.00014 -0.02744]';
rpy=[1.5708 -1.5708 -3.14159];
#com = (rotZ(rpy(3))*rotY(rpy(2))*rotX(rpy(1)))'*com;
I = [ixx, ixy, ixz; ixy, iyy, iyz; ixz, iyz, izz];
I = I + mass * com' * com * eye(3) + com * com';
I = (rotZ(rpy(3))*rotY(rpy(2))*rotX(rpy(1)))'*I*(rotZ(rpy(3))*rotY(rpy(2))*rotX(rpy(1)));
LElbowYaw = I

