frameLength = 0.5;  % length of axes. (I used meters in DH, so 0.1 = 10cm)
frameWidth = 2;     % width of the frame quivers

trace_position = true;
trace_time = 30;

view_azimuth = 45 + 90;          % azimuth for the view angle
view_rot_speed = 10;
view_elevation = 30;              % camera elevation
view_width = 10.0;                   % width of the workspace
view_center = [0, 0, 0];        % center of the view frame
center_around_robot = true; % If you turn this on, the graph will be centered around the robot.

freq = 20;
dt = 1/freq;
draw_interval = 2;

joy = vrjoystick(1);

use_linear = false;

% LQR matrices:
eps = 1e-8;
Q = diag([1,1,1,10,10,1,1,1,1,1,1,1]'*10);
R = diag([1,1,1,1,1,1]'*0.01);
u_sat = [40, 40, 40, 40, 40, 40]';

close all;
figure;
% hack: make the figure full screen for maximum video resolution:
set(gcf, 'Units', 'Normalized', 'OuterPosition', [0.25 0.1 0.5 0.8]);
n = 0;
x = zeros(12,1);
x_ned = zeros(12,1);
[A, B, G, A_sym, G_sym] = EoM_Linear(@EoM_6DoF, [0,0,0,0,0,pi/4,0,0,0,0,0,0]');

A_fn = matlabFunction(A_sym);
G_fn = matlabFunction(G_sym);

states = zeros(12,0);
inputs = zeros(6,0);
while (true)
    n = n+1;
    tic;
    
    v_ref = toNed(joy2v(joy));
    u_ref = toNed(joy2u(joy));
    pos_ref = [-2, .5, 1, 0, 0, pi/2]';
    x_ref = [pos_ref;v_ref];
    x_ref_cell = num2cell(x_ref);
    A = A_fn(x_ref_cell{4:12});
    x_ned_cell = num2cell(x_ned);
    G = G_fn(x_ned_cell{4:5});
    K = lqr(A, B, Q, R);
    err = x_ned - x_ref;
    u_req = -K * err + G;
    u_ned = min(abs(u_req), u_sat) .* sign(u_req);
    %dxdt = A*x_ned + B*u_ned;
    dxdt = EoM_6DoF(x_ned, u_ned);
    x_ned = x_ned + dxdt * dt;
    
    x(1:6) = fromNed(x_ned(1:6));
    x(7:12) = fromNed(x_ned(7:12));
    states(:,n) = x;
    inputs(:,n) = u;
    
    toc
    if (mod(n,draw_interval) == 0)
        
        % Draw 3d quiver
        clf;
        o = x(1:3);
        b = eul2rotm(x(6:-1:4)', 'ZYX') * frameLength;
        quiver3(o(1), o(2), o(3), b(1,1), b(2,1), b(3,1), 'color', 'r', 'linewidth', frameWidth);
        hold on
        quiver3(o(1), o(2), o(3), b(1,2), b(2,2), b(3,2), 'color', 'g', 'linewidth', frameWidth);
        quiver3(o(1), o(2), o(3), b(1,3), b(2,3), b(3,3), 'color', 'b', 'linewidth', frameWidth);
        
        % Draw the trace:
        s = max(1, ceil(n - trace_time*freq));
        plot3(states(1,s:end), states(2,s:end), states(3,s:end), 'color', 'm');
            
        % Set axes and view:
        if center_around_robot
            vc = o;
        else
            vc = view_center;
        end
        zx = vc(1);
        zy = vc(2);
        zz = vc(3);
        axis equal
        xlim([zx - view_width/2, zx + view_width/2]) 
        ylim([zy - view_width/2, zy + view_width/2]) 
        zlim([zz - view_width/2, zz + view_width/2])
        view_azimuth = mod(view_azimuth + view_rot_speed * dt, 360);
        view(view_azimuth, view_elevation)
        xlabel('x')
        ylabel('y')
        zlabel('z')
        
    end
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