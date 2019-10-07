%compute the time derivative of a rotation matrix

syms t q1(t) q2(t) q3(t) q4(t) q1p q2p q3p q4p q1_dot q2_dot q3_dot q4_dot real

%% tait  bryan angles
%rotation about the x axis by q1
Rx = [1          0           0;
      0 cos(q1(t)) -sin(q1(t));
      0 sin(q1(t)) cos(q1(t))];
         
%rotation about the y axis by q2
Ry = [ cos(q2(t)) 0 sin(q2(t));
                0 1          0;
      -sin(q2(t)) 0 cos(q2(t))];

%rotation about the z axis by q3
Rz = [cos(q3(t)) -sin(q3(t)) 0;
      sin(q3(t))  cos(q3(t)) 0;
               0           0 1];
%transformation from Q to S
R_tot = Rz * Ry * Rx;


%% Gripper frame
R_tot_dot = diff(R_tot,t);

R_tot_dot = subs(R_tot_dot, diff(q1(t), t), q1_dot);
R_tot_dot = subs(R_tot_dot, diff(q2(t), t), q2_dot);
R_tot_dot = subs(R_tot_dot, diff(q3(t), t), q3_dot);
R_tot_dot = subs(R_tot_dot, diff(q4(t), t), q4_dot);

R_tot_dot = subs(R_tot_dot, q1(t), q1p);
R_tot_dot = subs(R_tot_dot, q2(t), q2p);
R_tot_dot = subs(R_tot_dot, q3(t), q3p);
R_tot_dot = subs(R_tot_dot, q4(t), q4p);

R_tot = subs(R_tot, q1(t), q1p);
R_tot = subs(R_tot, q2(t), q2p);
R_tot = subs(R_tot, q3(t), q3p);
R_tot = subs(R_tot, q4(t), q4p);

omegaG = simplify(R_tot_dot * R_tot');


