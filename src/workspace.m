
init_q = [0 0 0 0 0 0]';
init_qdot = [100 0 -2 0 0 0]';
init_flap = [0 0 0 0];
init_thrust = 100;

uav = UAV(init_q, init_qdot, init_flap, init_thrust);
index = 1;
for i = -15:0.1:15
    %uav.flapAngles(1) = i;
    %uav.flapAngles(2) = i;
    %uav.flapAngles(3) = i;
    %uav.flapAngles(4) = i;
    
    ncforces(:,index) = uav.calculateNCForces;
    index = index + 1;
end

plot(-15:0.1:15, ncforces(:,:));
legend('Fx', 'Fy', 'Fz', 'Mx', 'My', 'Mz', 'Location', 'SouthEast');
xlabel('Flap Angle (deg)');
ylabel('Nonconservative Forces (N, Nm)');
title('Nonconservative Forces with Asymmetrically Varying Wing Flap Angle');


global path
figure;
plot(1:length(path(2,2:end)), path(1:3,2:end))
legend('X', 'Y', 'Z');
xlabel('Time Steps');
ylabel('Position');
title('Position of UAV Center of Mass Over Time');

figure;
plot(1:length(path(2,2:end)), path(4:6,2:end))
legend('Phi', 'Theta', 'Psi');
xlabel('Time Steps');
ylabel('Orientation');
title('Orientation of UAV Over Time');
