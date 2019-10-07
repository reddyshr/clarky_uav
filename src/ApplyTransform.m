function out = ApplyTransform (g, fv)
% ApplyTransform : Applies the transform specified by the matrix g to the
% coordinates of the patch object defined by the structure fv

% Extract the vertices
P = fv.vertices';

% Add a 1 to make the coordinates homogenous
P(end+1,:) = 1;

% Apply the transformation
P2 = g * P;

% Make the output structure
out = fv;

% Remove the last row of 1s in the homogenous output
out.vertices = P2(1:end-1,:)';