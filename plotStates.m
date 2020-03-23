function [] = plotStates(states, dt)
%PLOTSTATES Summary of this function goes here
%   Detailed explanation goes here
t = dt:dt:size(states,2)*dt;
plot(t,states(1,:));
hold on
plot(t,states(2,:));
plot(t,states(3,:));
legend('x','y','z');
end

