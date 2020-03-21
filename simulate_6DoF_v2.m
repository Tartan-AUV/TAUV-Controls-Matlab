clear;
close all;

% LQR matrices:
eps = 1e-8;
Q = diag([1,1,1,10,10,1,1,1,1,1,1,1]'*10);
R = diag([1,1,1,1,1,1]'*0.01);
u_sat = [40, 40, 40, 40, 40, 40]';

% System dynamics in ENU:
[A_sym, B_sym, G_sym, x_dot_sym] = EoM_Collect_Enu(@EoM_6DoF);
A_fn = matlabFunction(A_sym);
B_fn = matlabFunction(B_sym);
G_fn = matlabFunction(G_sym);
x_dot_fn = matlabFunction(x_dot_sym);

freq = 10;
dt = 1/freq;

vis = Visualize6DoF(dt);
joy = vrjoystick(1);

n = 0;
x = zeros(12,1);

while true
    tic;
    n = n + 1;
    
    v_ref = joy2v(joy);
    u_ref = joy2u(joy);
    pos_ref = [-2, -.5, -1, 0, 0, -pi/2]';
    x_ref = [pos_ref;v_ref];
    
    % Convert to cell matrices to simplify function calls:
    %  https://www.mathworks.com/matlabcentral/answers/8266-convert-array-to-argument-list
    x_ref_cell = num2cell(x_ref);
    x_cell = num2cell(x);
    
    % Compute Linear System:
    A = A_fn(x_cell{4:12});
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
    x = x + x_dot * dt
    
    % Update Visualizer:
    vis.setRobotState(x, n);
    vis.setReferenceState(x_ref);
    vis.showFrame();
    
    % Wait for dt to make this real-time:
    pause(max(0, toc - dt));
end

function u = joy2u(joy)
    u = zeros(6,1);
    u(1) = -axis(joy, 2)                     * 10;
    u(2) = -axis(joy, 1)                     * 10;
    u(3) = (axis(joy,3) - axis(joy,6))/2     * 10;
    u(4) = axis(joy,7)                       * 10;
    u(5) = -axis(joy,8)                      * 10;
    u(6) = -axis(joy, 4)                     * 10;
    
    u = u .* double(abs(u) > 0.1);
end

function u = joy2v(joy)
    u = zeros(6,1);
    u(1) = -axis(joy, 2)                     * 1;
    u(2) = -axis(joy, 1)                     * 1;
    u(3) = (axis(joy,3) - axis(joy,6))/2     * 1;
    u(4) = axis(joy,7)                       * 1;
    u(5) = -axis(joy,8)                      * 1;
    u(6) = -axis(joy, 4)                     * 1;
    
    u = u .* double(abs(u) > 0.1);
end