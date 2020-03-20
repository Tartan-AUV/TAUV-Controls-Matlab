classdef Visualize6DoF < handle
    %VISUALIZE6DOF Summary of this class goes here
    %   Detailed explanation goes here
    
    properties (Constant = true)
        frameLength = 0.5;  % length of axes. (I used meters in DH, so 0.1 = 10cm)
        frameWidth = 2;     % width of the frame quivers

        trace_position = true;
        trace_time = 30;
        
        view_azimuth_init = 45 + 90;
        view_rot_speed = 10;             % degrees per second
        view_elevation = 30;              % camera elevation
        view_width = 10.0;                   % width of the workspace
        view_center = [0, 0, 0];        % center of the view frame
        center_around_robot = true; % If you turn this on, the graph will be centered around the robot.
    end
    properties
        dt;
        n;
        referenceState;
        robotState;
        robotStates;
        fig;
        view_azimuth          % azimuth for the view angle
        freq;
    end
    
    methods
        function obj = Visualize6DoF(dt)
            %VISUALIZE6DOF Construct an instance of this class
            %   Detailed explanation goes here
            obj.dt = dt;
            obj.freq = 1/dt;
            obj.fig = figure();
            obj.referenceState = zeros(12,1);
            obj.robotState = zeros(12,1);
            obj.robotStates = zeros(12,0);
            obj.view_azimuth = obj.view_azimuth_init;
            set(gcf, 'Units', 'Normalized', 'OuterPosition', [0.25 0.1 0.5 0.8]);
        end
        
        function [] = setReferenceState(obj,x)
            %METHOD1 Summary of this method goes here
            %   Detailed explanation goes here
            obj.referenceState = x;
        end
        
        function [] = setRobotState(obj,x,n)
            %METHOD1 Summary of this method goes here
            %   Detailed explanation goes here
            obj.n = n;
            obj.robotState = x;
            obj.robotStates(:,obj.n) = x;
        end
        
        function [] = showFrame(obj)
            % Draw 3d quiver
            oldfig = gcf;
            set(0, 'CurrentFigure', obj.fig)
            clf;
            
            % Draw current state:
            x = obj.robotState;
            o = x(1:3);
            b = eul2rotm(x(6:-1:4)', 'ZYX') * obj.frameLength;
            quiver3(o(1), o(2), o(3), b(1,1), b(2,1), b(3,1), 'color', 'r', 'linewidth', obj.frameWidth);
            hold on
            quiver3(o(1), o(2), o(3), b(1,2), b(2,2), b(3,2), 'color', 'g', 'linewidth', obj.frameWidth);
            quiver3(o(1), o(2), o(3), b(1,3), b(2,3), b(3,3), 'color', 'b', 'linewidth', obj.frameWidth);
            
            title('3d visualizer')
            % Draw reference state:
            %x = obj.referenceState;
            %o = x(1:3);
            %b = eul2rotm(x(6:-1:4)', 'ZYX') * obj.frameLength;
            %h1 = quiver3(o(1), o(2), o(3), b(1,1), b(2,1), b(3,1), 'color', 'r', 'linewidth', obj.frameWidth);
            %h2 = quiver3(o(1), o(2), o(3), b(1,2), b(2,2), b(3,2), 'color', 'g', 'linewidth', obj.frameWidth);
            %h3 = quiver3(o(1), o(2), o(3), b(1,3), b(2,3), b(3,3), 'color', 'b', 'linewidth', obj.frameWidth);
            %h1.LineStyle = '--';
            %h2.LineStyle = '--';
            %h3.LineStyle = '--';
            
            % Draw the trace:
            s = max(1, ceil(obj.n - obj.trace_time*obj.freq));
            plot3(obj.robotStates(1,s:end), obj.robotStates(2,s:end), obj.robotStates(3,s:end), 'color', 'm');

            % Set axes and view:
            if obj.center_around_robot
                vc = o;
            else
                vc = obj.view_center;
            end
            zx = vc(1);
            zy = vc(2);
            zz = vc(3);
            axis equal
            xlim([zx - obj.view_width/2, zx + obj.view_width/2]) 
            ylim([zy - obj.view_width/2, zy + obj.view_width/2]) 
            zlim([zz - obj.view_width/2, zz + obj.view_width/2])
            obj.view_azimuth = mod(obj.view_azimuth + obj.view_rot_speed * obj.dt, 360);
            view(obj.view_azimuth, obj.view_elevation)
            xlabel('x')
            ylabel('y')
            zlabel('z')
            
            set(0, 'CurrentFigure', oldfig)
        end
    end
end

