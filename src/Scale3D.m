function g = Scale3D (sx, sy, sz)
% Return a transformation that scales along the x and y dimensions
g = [sx 0 0 0; 0 sy 0 0; 0 0 sz 0; 0 0 0 1];