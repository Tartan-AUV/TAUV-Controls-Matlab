clear;
close all

dt = 0.1;
x = zeros(12,1);

% LQR matrices:
eps = 1e-8;
Q = diag([10,10,10,100,100,10,.1,.1,.1,.1,.1,.1]'*10);
R = diag([1,1,1,1,1,1]'*0.01);
u_sat = [40, 40, 40, 40, 40, 40]';
na = 10000;
x_sat = [na, na, na, na, na, na, 1, 1, 1, 1, 1, 1]';

% System dynamics in ENU:
[A_sym, B_sym, G_sym, x_dot_sym] = EoM_Collect_Enu(@EoM_6DoF);
A_fn = matlabFunction(A_sym);
B_fn = matlabFunction(B_sym);
G_fn = matlabFunction(G_sym);
x_dot_fn = matlabFunction(x_dot_sym);

mpc = MpcTrajectoryPlanner(A_sym, double(B_sym), Q, R, Q*10, ...
 [-x_sat, x_sat], [-u_sat, u_sat], 10, dt);

vis = Visualize6DoF(dt);

pos_ref = [-4, -2, -2, 0, 0, -pi]';
v_ref = zeros(6,1);
x_target = [pos_ref;v_ref];

[u_traj, x_traj] = mpc.computeTrajectory(x, x_target);

for n = 1:mpc.N
    tic;
    
    % Move reference:
    x_ref = x_traj(:,n);
    x_ref_cell = num2cell(x_ref);
    
    % Compute Linear System:
    x_cell = num2cell(x);
    A = A_fn(x_ref_cell{4:12});
    B = B_fn();
    G = G_fn(x_cell{4:5});
    
    % Compute Control Input:
    K = lqr(A, B, Q, R);
    err = x - x_ref;
    u_req = -K * err + G;
    u = min(abs(u_req), u_sat) .* sign(u_req);
    u_cell = num2cell(u);
    
    % Simulate System:
    x_dot = x_dot_fn(u_cell{:}, x_cell{4:12});
    x = x + x_dot * dt;
    
    % Update Visualizer:
    vis.setRobotState(x, n);
    vis.setReferenceState(x_ref);
    vis.showFrame();
    
    pause(max(0, toc - dt));
end