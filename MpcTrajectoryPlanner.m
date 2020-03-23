classdef MpcTrajectoryPlanner < handle
    %MPCTRAJECTORYPLANNER Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        N
        A_fn
        B
        L_u_bar
        L_x_bar
        b_u_bar
        b_x_bar
        Q_bar
        R_bar
    end
    
    methods
        function obj = MpcTrajectoryPlanner(A_sym, B, Q, R, S, x_constraints, u_constraints, horizon, dt)
            %MPCTRAJECTORYPLANNER Construct an instance of this class
            %   Detailed explanation goes here
            %
            % constraints(:,1) is minimums
            % constraints(:,2) is maximums
            %
            
            N = ceil(horizon / dt);
            obj.N = N;
            obj.A_fn = matlabFunction(A_sym * dt + eye(12));
            obj.B = B;
            
            L_x = [eye(12); -eye(12)];
            b_x = [x_constraints(:,2); -x_constraints(:,1)];
            L_u = [eye(6); -eye(6)];
            b_u = [u_constraints(:,2); -u_constraints(:,1)];
            obj.L_u_bar = bigblockdiag(L_u, N);
            obj.L_x_bar = bigblockdiag(L_x, N+1);
            obj.b_u_bar = bigstack(b_u, N);
            obj.b_x_bar = bigstack(b_x, N+1);
            
            obj.Q_bar = bigblockdiag(Q, obj.N+1);
            obj.Q_bar(end-11:end,end-11:end) = S;
            obj.R_bar = bigblockdiag(R, obj.N);
            
        end
        
        function [u_traj, x_traj] = computeTrajectory(obj, x, x_ref)
            %METHOD1 Summary of this method goes here
            %   Detailed explanation goes here
            x_cell = num2cell(x);
            A = obj.A_fn(x_cell{4:12});
            
            A_stack = build_a_bar(A, obj.N+1);
            B_bar = build_b_bar(A, obj.B, obj.N+1);
            
            f_bar = A_stack * x;
            G_bar = bigstack(x_ref, obj.N+1);
            H = B_bar' * obj.Q_bar * B_bar + obj.R_bar;
            C = f_bar' * obj.Q_bar * B_bar - G_bar' * obj.Q_bar * B_bar;
            b = [obj.b_u_bar; obj.b_x_bar - obj.L_x_bar * f_bar];
            L = [obj.L_u_bar; obj.L_x_bar * B_bar];
            tic
            evalc('U = quadprog(H, C, L, b)');
            toc
            X = f_bar + B_bar * U;
            
            u_traj = reshape(U,6,obj.N);
            x_traj = reshape(X,12,obj.N+1);
        end
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

function A_bar = build_a_bar(A, n)
    rm = size(A,1);
    cm = size(A,2);
    A_bar = zeros(rm*n,cm);
    
    for i = 1:n
        A_bar(rm*(i-1)+1:rm*i,:) = A^(i-1);
    end
end

function B_bar = build_b_bar(A, B, n)
    rm = size(B,1);
    cm = size(B,2);
    B_bar = zeros(rm*n,cm*(n-1));
    for r = 1:n
        for c = 1:n-1
            order = r-c-1;
            if order < 0
                B_bar(rm*(r-1)+1:rm*r, cm*(c-1)+1:cm*c) = zeros(size(B));
            else
                B_bar(rm*(r-1)+1:rm*r, cm*(c-1)+1:cm*c) = A^order * B;
            end
        end
    end
end