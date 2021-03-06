function [x_dot, G] = EoM_6DoF(params, x, u)
% Terms:
% - CO: Center of Orientation. Origin datum for the sub. (origin of
% base_link)
% - CoG: Center of Gravity. Defined w.r.t CO
% - CoB: Center of Buoyancy. Defined w.r.t CO
% - Moments of inertia: Inertia moments of the dry vehicle about the CoG.
% 
% 
% State variables:
% - x = [n, v]. Full state in NED.
% - u: wrench input about CO
% - v1: Linear velocities in body frame about [x,y,z]
% - v2: Angular velicities in body frame about [x,y,z]
% - v = [v1; v2]
% 
% - n1 = [x,y,z] position defined in NED (north, east, down)
% - n2 = [phi, theta, psi] orientation defined  in NED as extrinsic
% applied euler angles about x, y, z axes respectively. (Roll, Pitch, Yaw)
% ('xyz' extrinsic euler angles, or 'ZYX' intrinsic euler angles: [psi, theta, phi].)
% - n = [n1; n2]
% 
% 
% 
% Matrices:
% 
% - J(n2): Transformation matrix from body frame velocities v to n' (NED
% frame velocities). Function of n2. Also note: J = [J1, 0; 0, J2];
% - I: Inertia tensor defined in body frame about CoG
% - M_rb: rigid-body mass matrix in body frame
% - C_rb(v): rigid-body coriolis matrix in body frame. Function of v
% - M_added: Added mass matrix
% - C_added(v): Added coriolis matrix. Function of velocity.
%
%

%%%%%%%%%%%%%%%%%%
% Configuration: %
%%%%%%%%%%%%%%%%%%

m = params.m; % Vehicle Mass in air
b = params.b; % Vehicle Buoyancy (Mass of displaced water)
r_G = diag([1,-1,-1])*params.r_G; % CoG location relative to CO (NED)
r_B = diag([1,-1,-1])*params.r_B; % CoB location relative to CO (NED)

% Moments of inertia about CoG:
Ixx = params.Ixx;
Iyy = params.Iyy;
Izz = params.Izz;
Ixy = params.Ixy;
Ixz = params.Ixz;
Iyz = params.Iyz;

% Linear damping coefficients: (About CO)
D_x = params.D_x;
D_y = params.D_y;
D_z = params.D_z;
D_yaw = params.D_yaw;
D_pitch = params.D_pitch;
D_roll = params.D_roll;

% Quadratic damping coefficients: (About CO)
D2_x = params.D2_x;
D2_y = params.D2_y;
D2_z = params.D2_z;
D2_yaw = params.D2_yaw;
D2_pitch = params.D2_pitch;
D2_roll = params.D2_roll;

% Added mass coefficients: (About CO)
Ma_x = params.Ma_x;
Ma_y = params.Ma_y;
Ma_z = params.Ma_z;
Ma_yaw = params.Ma_yaw;
Ma_pitch = params.Ma_pitch;
Ma_roll = params.Ma_roll;

%%%%%%%%%%%%%%%%%%%%%%%%
% Equations of Motion: %
%%%%%%%%%%%%%%%%%%%%%%%%

% Extract parts of state:
n = x(1:6);
n1 = n(1:3);
n2 = n(4:6);
v = x(7:12);
v1 = v(1:3);
v2 = v(4:6);

% Build EoM Matrices:
J = buildEulerMatrix(n2);
I = buildInertiaTensor(Ixx, Iyy, Izz, Ixy, Ixz, Iyz);
M_rb = buildMassMatrix(m, r_G, I);
C_rb = buildCoriolisMatrix(m, r_G, I, v1, v2);
D_lin = diag([D_x, D_y, D_z, D_roll, D_pitch, D_yaw]);
D_quad = diag([D2_x, D2_y, D2_z, D2_roll, D2_pitch, D2_yaw]) * abs(v);
[M_added, C_added] = buildAddedMassCoriolisMatrices(v, Ma_x, Ma_y, Ma_z, Ma_roll, Ma_pitch, Ma_yaw);
G = buildGravityMatrix(m, b, r_G, r_B, n2);

% Property 4.1: (Chin 2013, p 139)
M = M_rb + M_added;
C = C_rb + C_added;
D = D_lin + D_quad;
M_inv = inv(M);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Nonlinear state-space function: %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Eq 4.24a (Chin 2013, p 138)
f_x = [zeros(6), J; zeros(6), -M_inv*(C+D)] * [n; v] + [zeros(6,1); -M_inv*G];
g_u = [zeros(6,1); M_inv*u];

x_dot = f_x + g_u;

end

function [J, J1, J2] = buildEulerMatrix(n2)
    % Note: Due to the use of euler angle representation, J becomes
    % undefined when theta = +/- 90 degrees.
    % TODO: switch to a quaternion representation!
    phi = n2(1);
    theta = n2(2);
    psi = n2(3);
    
    cphi = cos(phi);
    sphi = sin(phi);
    cth = cos(theta);
    sth = sin(theta);
    tth = tan(theta);
    cpsi = cos(psi);
    spsi = sin(psi);
    
    % Prevent singularity condition
    %eps = 0.001;
    %cth = max(abs(cth), eps)*sign(cth);
    
    % Eq 4.17 (Chin 2013, p 136)
    J1 = [cpsi*cth, -spsi*cphi + cpsi*sth*sphi, spsi*sphi + cpsi*cphi*sth;
        spsi*cth, cpsi*cphi + sphi*sth*spsi, -cpsi*sphi + sth*spsi*cth;
        -sth, cth*sphi, cth*cphi];
    % J1 = eul2rotm([phi,theta,psi], 'yxz')';
    
    % NOTE: There is an error in Chin 2013. J2(2,2) = cphi, not sphi!
    J2 = [1, sphi*tth, cphi*tth;
        0, cphi, -sphi;
        0, sphi/cth, cphi/cth];
    
    J = [J1, zeros(3); zeros(3), J2];
end

function I = buildInertiaTensor(Ixx, Iyy, Izz, Ixy, Ixz, Iyz)
    % Eq 4.3b: (Chin 2013, p 132)
    I = [Ixx, -Ixy, -Ixz;
        -Ixy,  Iyy, -Iyz;
        -Ixz, -Iyz,  Izz];
end

function M_rb = buildMassMatrix(m, r_G, I)
    % Eq 4.6: (Chin 2013, p 132)
    M_rb = [m * eye(3), -m*skew(r_G);
            m * skew(r_G), I];
end

function C_rb = buildCoriolisMatrix(m, r_G, I, v1, v2)
    % Eq 4.10: (Chin 2013, p 134)
    C_rb = [zeros(3),   -m*skew(v1) - m*skew(v2)*skew(r_G);
            -m*skew(v1) + skew(r_G)*skew(v2),   -skew(I*v2)];
end

function [M_added, C_added] = buildAddedMassCoriolisMatrices(v, Ma_x, Ma_y, Ma_z, Ma_yaw, Ma_pitch, Ma_roll)
    % eq 4.32: (Chin 2013, p 143)
    M_added = -diag([Ma_x, Ma_y, Ma_z, Ma_yaw, Ma_pitch, Ma_roll]);
    
    % eq 4.33: (Chin 2013, p 143)
    % TODO: verify correctness!
    C_added = ...
        [0, 0, 0, 0, -Ma_z * v(3), Ma_y * v(2);
         0, 0, 0, Ma_z * v(3), 0, -Ma_x * v(1);
         0, 0, 0, -Ma_y * v(2), Ma_x * v(1), 0;
         0, -Ma_z * v(3), Ma_y * v(2), 0, -Ma_roll * v(6), Ma_pitch * v(5);
         Ma_z * v(3), 0, -Ma_x * v(1), Ma_roll * v(6), 0, -Ma_yaw * v(4);
         -Ma_y * v(2), Ma_x * v(1), 0, -Ma_pitch * v(5), Ma_yaw * v(4), 0];
end

function G = buildGravityMatrix(m, b, r_G, r_B, n2)
    g = 9.81;
    W = m*g;
    B = b*g;
    
    phi = n2(1);
    theta = n2(2);
    cph = cos(phi);
    sph = sin(phi);
    cth = cos(theta);
    sth = sin(theta);
    
    x_G = r_G(1);
    y_G = r_G(2);
    z_G = r_G(3);
    x_B = r_B(1);
    y_B = r_B(2);
    z_B = r_B(3);
    
    % Eq 4.54 (Chin 2013, p 180)
    G = [(W-B) * sth;
         -(W-B) * cth*sph;
         -(W-B) * cth*cph;
         -(y_G*W - y_B*B)*cth*cph + (z_G*W - z_B*B)*cth*sph;
         (z_G*W - z_B*B)*sth + (x_G*W - x_B*B)*cth*cph;
         -(x_G*W - x_B*B)*cth*sph - (y_G *W - y_B*B)*sth;];
end

function S = skew(x)
    % Chin 2013, p 133.
    % Defined as S(x)*y = cross(x,y)
    S = [0 -x(3) x(2) ; x(3) 0 -x(1) ; -x(2) x(1) 0 ];
end