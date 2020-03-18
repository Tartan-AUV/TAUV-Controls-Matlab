function [A, B, A_sym] = EoM_Linear(EoM_fn, x_ref)
%EOM_6DOF_LINEARIZE Return A and B matrices for linearized EoM
%   
%   For a given nonlinear equations of motion in the form:
%       x_dot = f(x, u)
%   where:
%       x is a 12x1 state vector
%       u is a 6x1 input vector
%       x_dot is the 12x1 derivative of x
%
%   Returns the linearized EoM about x_ref in the form:
%       x_dot = A(x_ref)*x + B*u
%
%   Outputs:
%       A: Linearized 12x12 A matrix about x_ref
%       B: Linearized 12x6 B matrix
%       A_sym: Linearized 12x12 A matrix with symbolic x_ref
%

x = sym('x', [12,1]);
u = sym('u', [6,1]);
x_dot = EoM_fn(x,u);

A_sym = jacobian(x_dot, x);
A = subs(A_sym, x, x_ref);

B = jacobian(x_dot, u);
end