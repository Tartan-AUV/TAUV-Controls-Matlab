function [params] = auvParamsAlbatross()
%AUVPARAMS Summary of this function goes here
%   Detailed explanation goes here

params.vehicle_name = "albatross";

params.m = 10; % Vehicle Mass in air
params.b = 10.5; % Vehicle Buoyancy (Mass of displaced water)
params.r_G = [0;0;0]; % CoG location relative to CO (standard coords)
params.r_B = [0;0;0];% CoB location relative to CO (standard coords)

params.Ixx = 10;
params.Iyy = 10;
params.Izz = 10;
params.Ixy = 0;
params.Ixz = 0;
params.Iyz = 0;

% Linear damping coefficients: (About CO)
params.D_x = 10;
params.D_y = 10;
params.D_z = 10;
params.D_roll = 10;
params.D_pitch = 10;
params.D_yaw = 10;

% Quadratic damping coefficients: (About CO)
params.D2_x = 0;
params.D2_y = 0;
params.D2_z = 0;
params.D2_roll = 0;
params.D2_pitch = 0;
params.D2_yaw = 0;

% Added mass coefficients: (About CO)
params.Ma_x = 0;
params.Ma_y = 0;
params.Ma_z = 0;
params.Ma_roll = 0;
params.Ma_pitch = 0;
params.Ma_yaw = 0;

end

