
[~, ~, ~, x_dot_sym] = EoM_Collect_Enu(@EoM_6DoF);
x_dot_fn = matlabFunction(x_dot_sym);

dt = 0.1;
horizon = 10;

N = ceil(horizon / dt);

Q = diag([1,1,1,10,10,1,1,1,1,1,1,1]'*10);
R = diag([1,1,1,1,1,1]'*0.01);
pos_ref = [-2, -.5, -1, 0, 0, -pi/2]';
v_ref = zeros(6,1);
x_ref = [pos_ref;v_ref];

Q_bar = bigblockdiag(Q, N);
R_bar = bigblockdiag(R, N);
x_ref_bar = bigstack(x_ref, N);

options = optimoptions('fmincon','MaxIterations',100000, 'MaxFunctionEvaluations', 100000, ...
    'OptimalityTolerance', 1e-10);
tic
[u_traj, fval] = fmincon(@(u_traj) costFn(x, u_traj, N, dt, x_dot_fn, x_ref_bar, Q_bar, R_bar), zeros(6*N,1), ...
    [], [], [], [], [], [], [], options);
toc
fval

x_traj = simNonlin(x, u_traj, N, dt, x_dot_fn);
vis = Visualize6DoF(dt);
for i = 1:N
    tic
    vis.setRobotState(x_traj(12*i-11:12*i), i);
    vis.setReferenceState(x_ref);
    vis.showFrame();
    pause(max(0, toc - dt));
end

function cost = costFn(x, u_traj, N, dt, x_dot_fn, x_ref_bar, Q_bar, R_bar)
    x_traj = simNonlin(x, u_traj, N, dt, x_dot_fn);
    cost = (x_traj-x_ref_bar)'*Q_bar*(x_traj-x_ref_bar) + u_traj'*R_bar*u_traj;
end

function x_traj = simNonlin(x, u_traj, N, dt, x_dot_fn)
    x_traj = zeros(12*N,1);
    for i = 1:N
        u = u_traj(6*i-5:6*i);
        x = x + dt * x_dot_fn(...
            u(1), ...
            u(2), ...
            u(3), ...
            u(4), ...
            u(5), ...
            u(6), ...
            x(4), ...
            x(5), ...
            x(6), ...
            x(7), ...
            x(8), ...
            x(9), ...
            x(10), ...
            x(11), ...
            x(12));
        x_traj(12*i-11:12*i) = x;
    end
end

function blk = bigblockdiag(M, n)
    blk = zeros(size(M)*n);
    
    rm = size(M,1);
    cm = size(M,2);
    
    for i = 1:n
        blk(rm*(i-1)+1:rm*i, cm*(i-1)+1:cm*i) = M;
    end
end

function stk = bigstack(M,n)
    rm = size(M,1);
    cm = size(M,2);
    stk = zeros(rm*n,cm);
    
    for i = 1:n
        stk(rm*(i-1)+1:rm*i,:) = M;
    end
end