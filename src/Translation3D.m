function g = Translation3D (tx, ty, tz)
% Produce a rigid transformation corresponding to a translation
g = [eye(3) [tx; ty; tz]; 0 0 0 1];