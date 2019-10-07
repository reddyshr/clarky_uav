%symbolic lagrangian calculator
%% declare variables
syms x y z phi theta psi x_dot y_dot z_dot phi_dot theta_dot psi_dot Ixx Iyy Izz Iyz Ixz Ixy ...
    x_ddot y_ddot z_ddot phi_ddot theta_ddot psi_ddot Fx Fy Fz Mx My Mz m  g real

%state vector consists of x y z position of the UAV measured by the
%inertial world frame and the tait-bryan angles that represent the
%orientation of the uav wrt to the world frame

state = [x y z phi theta psi]';
state_dot = [x_dot y_dot z_dot phi_dot theta_dot psi_dot]';
state_ddot = [x_ddot y_ddot z_ddot phi_ddot theta_ddot psi_ddot]';
%% Inertia tensor
% I = [Ixx Ixy Ixz;
%      Ixy Iyy Iyz;
%      Ixz Iyz Izz];

I = [Ixx 0 0;
    0 Iyy 0;
    0 0 Izz];
%% Write the angular rates in terms of the state
p = phi_dot - psi_dot*sin(phi);
q = theta_dot*cos(phi) + psi_dot*sin(phi)*cos(theta);
r = -theta_dot*sin(phi) + psi_dot*cos(phi)*cos(theta);
omega = [p q r]';
%% Qnc nononservative forces and moments, lift and drag
Qnc = [Fx Fy Fz Mx My Mz]';

%% Angular kinetic engery

T2 = 0.5 * omega' * I * omega;
         
%% Linear kinetic engergy
T1 = 0.5 * m * (x_dot^2 + y_dot^2 + z_dot^2);

%% total kinetic
T = T1 + T2;

%% Potential Energy
V = - m * g * z;

%% Lagrangian
L = T - V;

dLdq = [diff(L, x); diff(L, y); diff(L, z); diff(L, phi); diff(L, theta); diff(L, psi)];

dLdq_dot = [  diff(L, x_dot);     diff(L, y_dot);   diff(L, z_dot);
            diff(L, phi_dot); diff(L, theta_dot); diff(L, psi_dot)];

ddt_dLdq_dot = diff(dLdq_dot, x)*x_dot + diff(dLdq_dot, y)*y_dot + diff(dLdq_dot, z)*z_dot + ...
        diff(dLdq_dot, phi)*phi_dot + diff(dLdq_dot, theta)*theta_dot + diff(dLdq_dot, psi)*psi_dot + ...
        diff(dLdq_dot, x_dot)*x_ddot + diff(dLdq_dot, y_dot)*y_ddot + diff(dLdq_dot, z_dot)*z_ddot + ...
        diff(dLdq_dot, phi_dot)*phi_ddot + diff(dLdq_dot, theta_dot)*theta_ddot + diff(dLdq_dot, psi_dot)*psi_ddot;
        

%% equations 
Eqs = 0 == simplify(ddt_dLdq_dot - dLdq - Qnc);

sols = (solve(Eqs, state_ddot));

solx = simplify(sols.x_ddot);
soly = simplify(sols.y_ddot);
solz = simplify(sols.z_ddot);
solphi = simplify(sols.phi_ddot);
soltheta = simplify(sols.theta_ddot);
solpsi = simplify(sols.psi_ddot);
%solphi = simplify(solve(Eqs(4), phi_ddot));
%soltheta = simplify(solve(Eqs(5), theta_ddot));
%solpsi = simplify(solve(Eqs(6), psi_ddot));