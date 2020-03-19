function x_Ned = toNed(x_Enu)
%ENU2NED Summary of this function goes here
%   Detailed explanation goes here
x_Ned = zeros(6,1);
x_Ned(1) = x_Enu(1);
x_Ned(2) = -x_Enu(2);
x_Ned(3) = -x_Enu(3);
x_Ned(4) = x_Enu(4);
x_Ned(5) = -x_Enu(5);
x_Ned(6) = -x_Enu(6);
end

