function dr = ode_2bp( ~, y, mu )
%ode_2bp ODE system for the two-body problem (Keplerian motion)
%
%
% INPUT:
% t[1] Time (can be omitted, as the system is autonomous) [T]
% y[6x1] State of the body ( rx, ry, rz, vx, vy, vz ) [ L, L/T ]
% mu[1] Gravitational parameter of the primary [L^3/T^2]
%
% OUTPUT:
% dr[6x1] Derivative of the state [ L/T^2, L/T^3 ]
%
% -------------------------------------------------------------------------

% Position and velocity
r = y(1:3);
v = y(4:6);

% Distance from the primary
rnorm = norm(r);

% Set the derivatives of the state
dr=[v
    (-mu/rnorm^3)*r];

end