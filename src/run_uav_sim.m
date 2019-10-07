clear all; clc; close all

%% Set up the figure

figure;
axis vis3d;
ax = gca;
ax.ZDir = 'reverse';
box on
axis ([-500 500 -500 500 -500 500]);
xlabel('X')
ylabel('Y')
zlabel('Z')

%% create the patch object for the UAV
fv = stlread('clarkY_UAV.STL');
fv_scale = ApplyTransform(Scale3D(1/25.4, 1/25.4, 1/25.4), fv);
% Rz = Rotation3D('x',90);
InsertToZero = Translation3D(-150, -6, -300); 
BodyToZero = [0 -1  0      0;
              0  0 -1   5.43;
              1  0  0 -61.56;
              0  0  0      1];
          
ZeroToBody = [BodyToZero(1:3,1:3)' -BodyToZero(1:3,1:3)'*BodyToZero(1:3,4); 0 0 0 1];
fv_scale = ApplyTransform(ZeroToBody*InsertToZero, fv_scale);


%% Initial Conditions

init_q = [0 0 -250 0 -10 0]'; %[x y z phi theta psi]
init_qdot = [100 0 -20 0 0 0]';%[xdot ydot zdot phidot thetadot psidot]
init_flap = [0 0 2 2];%[rightwing flap angle, left wing flap angle, right tail flap angle, left wing flap angle]
init_thrust = 1000;%thrust force

uav = UAV(init_q, init_qdot, init_flap, init_thrust);

tstep = 0.01;
tstart = 0.1;
tend = 15;
global path 
path = zeros(12,1);

%% apply the initial state to the patch object
initT = Translation3D(init_q(1), init_q(2), init_q(3));
initR = Rotation3D('z', init_q(6)) * Rotation3D('y', init_q(5)) * Rotation3D('x', init_q(4));
fv_start = ApplyTransform(initT*initR, fv_scale);

patch(fv_start,'FaceColor', 'Blue', 'FaceAlpha', 0.3)
%% run the simulation loop
for i = 1:length(tstart:tstep:tend)
    [new_q, new_qdot, a] = update_state([0 tstep], uav);
    uav.q = new_q;
    uav.qdot = new_qdot;
    uav.aoa = a;
    %path(:, i) = [uav.q; uav.qdot];
    
    % update the graphics
    Ht = Translation3D(new_q(1), new_q(2), new_q(3));
    Hr = Rotation3D('z', new_q(6)) * Rotation3D('y', new_q(5)) * Rotation3D('x', new_q(4));
    
    cla
    patch(ApplyTransform(Ht*Hr, fv_scale),'FaceColor', 'Blue', 'FaceAlpha', 0.3)
%     pause (0.01);
    drawnow;
end

