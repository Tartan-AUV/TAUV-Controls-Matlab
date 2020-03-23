% Test of single-shot mpc trajectory tracking
% (MPC trajectory generated once, then tracked with LQR).
clear;
close all

params = auvParamsAlbatross();

dt = 0.1;
x = zeros(12,1);
x_i = zeros(12,1);

% LQR matrices:
eps = 1e-8;
Q = diag([100,100,100,10,10,1,1,1,1,.1,.1,.1]'*100);
R = diag([1,1,1,1,1,1]'*0.01);
u_sat = [40, 40, 40, 40, 40, 40]';
na = 10000;
x_sat = [na, na, na, na, na, na, 1, 1, 1, 1, 1, 1]';

int_cap = [1,1,1,0,0,0]'*10;
Q_i = eye(6)*10;
err_int = zeros(6,1);

% System dynamics in ENU:
[A_sym, B_sym, G_sym, x_dot_sym] = EoM_Collect_Enu(@EoM_6DoF, params);
A_fn = matlabFunction(A_sym);
B_fn = matlabFunction(B_sym);
G_fn = matlabFunction(G_sym);
x_dot_fn = matlabFunction(x_dot_sym);

% Generate MPC trajectory
tic
mpc = MpcTrajectoryPlanner(A_sym, double(B_sym), Q, R, Q*10, ...
 [-x_sat, x_sat], [-u_sat, u_sat].*0.5, 10, dt);
toc

vis = Visualize6DoF(dt);

pos_ref = [5,1,-2, 0, 0, pi/2]';
v_ref = zeros(6,1);
x_target = [pos_ref;v_ref];

tic
[u_traj, x_traj] = mpc.computeTrajectory(x, x_target);
toc

for n = 1:10/dt
    tic;
    
    % Move reference:
    x_ref = x_traj(:,n);
    x_ref_cell = num2cell(x_ref);
    
    x_lin = (x_ref + x)./2;
    x_lin_cell = num2cell(x_lin);
    
    % Compute Linear System:
    x_cell = num2cell(x);
    A = A_fn(x_lin_cell{4:12});
    B = B_fn();
    G = G_fn(x_cell{4:5});
    
    % Compute Control Input:
    K = lqr([A,zeros(12,6);eye(6),zeros(6,12)], [B;zeros(6,6)], [Q,zeros(12,6);zeros(6,12),Q_i], R);
    err = x - x_ref;
    
    % Rotation error is more complex:
    %R_r = eul2rotm(x(6:-1:4)','ZYX');
    %R_g = eul2rotm(x_ref(6:-1:4)','ZYX');
    %R_err = R_g'*R_r;
    %err(6:-1:4) = rotm2eul(R_err, 'ZYX')';
    
    %err_int = err_int + err(1:6);
    err_int = abs(min(abs(err_int + err(1:6)), int_cap)) .* sign(err_int + err(1:6));
    u_req = -K * [err; err_int] + G;
    %u = min(abs(u_req), u_sat) .* sign(u_req);
    u = u_req;
    u_cell = num2cell(u);
    
    % Simulate System:
    x_dot = x_dot_fn(u_cell{:}, x_cell{4:12});
    x = x + x_dot * dt;
    
    % Update Visualizer:
    vis.setRobotState(x, n);
    vis.setReferenceState(x_ref, n);
    vis.showFrame();
    
    pause(max(0, toc - dt));
end

vis.showPlot();