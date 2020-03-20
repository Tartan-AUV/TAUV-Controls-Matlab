function [A_sym, B_sym, G_sym, x_dot_sym] = EoM_Collect_Enu(EoM_fn)
%EOM_COLLECT Summary of this function goes here
%   Detailed explanation goes here
    
    % Define NedTransformation matrix:
    NedTrans = diag([1,-1,-1,1,-1,-1]);
    NedTransFull = [NedTrans, zeros(6); zeros(6), NedTrans];

    % Convert to NED:
    x_ned = NedTransFull * sym('x', [12,1]);
    u_ned = NedTrans * sym('u', [6,1]);
    
    % Get symbolic nonlinear EoM and symbolic gravity vector:
    [x_dot_sym_ned, G_sym_ned] = EoM_fn(x_ned, u_ned);
    
    % Get symbolic linear matrices:
    A_sym_ned = jacobian(x_dot_sym_ned, sym('x', [12,1]));
    B_sym_ned = jacobian(x_dot_sym_ned, sym('u', [6,1]));
    
    % Convert back from NED:
    B_sym = NedTransFull * B_sym_ned;
    A_sym = NedTransFull * A_sym_ned;
    G_sym = NedTrans * G_sym_ned;
    x_dot_sym = NedTransFull * x_dot_sym_ned;
end