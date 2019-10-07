%stl read test and plot

%% Set up the figure

figure;
axis vis3d;
axis ([-10 2000 -10 500 -10 2000]);

%% 

fv = stlread('clarkY_UAV.STL');
fv_scale = ApplyTransform(Scale3D(0.1, 0.1, 0.1), fv);
patch(fv_scale,'FaceColor', 'Blue', 'FaceAlpha', 0.3)

for i = 1:100
    
    Rg = Rotation3D('y',4*i);
    
    cla
    patch(ApplyTransform(Rg, fv_scale),'FaceColor', 'Blue', 'FaceAlpha', 0.3)
    pause (0.1);
    drawnow;
end