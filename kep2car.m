function [r, v] = kep2car(a, e, i, OM, w, theta, mu)

% Trasformation from orbital (Keplerian) elements to Cartesian coordinates.
%
% [r, v] = parorb2rv (a, e, i, OM, w, theta, mu)
%
% Input arguments:
% ------------------------------------------------------------------------
% a         [1x1]       semi-major axis                    [km]
% e         [1x1]       eccentricity                       [-]
% i         [1x1]       inclination                        [deg]
% OM        [1x1]       RAAN                               [deg]
% w         [1x1]       argument of pericentre             [deg]
% theta     [1x1]       true anomaly                       [deg]
% mu        [1x1]       gravitational parameter            [km^3 / s^2]
%
% Output arguments:
% ------------------------------------------------------------------------
% r        [3x1]       position vector                 [km]
% v        [3x1]       velocity vector                 [km/s]


% Directions of the perifocal reference frame

pp = [1; 0; 0]; % radial direction
qq = [0; 1; 0]; % transversal direction
ww = [0; 0; 1]; % out of plane direction

p = a * (1 - e^2);
h = sqrt(mu * p);

rr_pf = (h^2 / mu) * (1 / (1 + e * cos(theta))) * (cos(theta) * pp + sin(theta) * qq);

vv_pf = (mu / h) * (-sin(theta) * pp + (e + cos(theta)) * qq);

% Definition of the Direction Cosine Matrices

R_OM = [cos(OM) sin(OM) 0;
        -sin(OM) cos(OM) 0;
                     0 0 1];

R_i = [1 0 0;
       0 cos(i) sin(i);
       0 -sin(i) cos(i)];

R_w = [cos(w) sin(w) 0;
        -sin(w) cos(w) 0;
                     0 0 1];

% Passage from perfical r.f. (pf) to Earth centred equatorial r.f. (Ece)

A_Ece_pf = R_w * R_i * R_OM; 

A_pf_Ece = A_Ece_pf';
                                                 

r = A_pf_Ece * rr_pf;
v = A_pf_Ece * vv_pf;

end
