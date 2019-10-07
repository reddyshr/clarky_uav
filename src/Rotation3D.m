function g = Rotation3D (axis, angle)
% Produce a rigid transformation corresponding to a rotation about X
c = cosd(angle);
s = sind(angle);

switch axis
    case 'x'
        R = [1 0 0; 0 c -s; 0 s c];
    case 'y'
        R = [c 0 s; 0 1 0; -s 0 c];
    case 'z'
        R = [c -s 0; s c 0; 0 0 1];
    otherwise
        R = eye(3);
end

g = [R zeros(3,1); zeros(1,3) 1];