function [new_q, new_qdot, a] = update_state(tspan, uav_obj)
%UNTITLED6 Summary of this function goes here
%   Detailed explanation goes here
global path
% update these to get the correct parameters of the uav_obj 
state = uav_obj.getState();
disp(state)
path = [path state];
%disp(uav_obj.aoa)
Qnc = uav_obj.calculateNCForces();

I = uav_obj.inertiaMatrix;
m = uav_obj.mass;
[ts, qs] = ode45(@(t,q) compute_dynamics(t,q,I, m, Qnc, uav_obj), tspan, state);
%% not sure how to update this state of the uav_object
new_q = qs(end,1:6)';
new_qdot = qs(end,7:12)';
a = uav_obj.updateAOA;
%uav_obj.q = new_q;
%uav_obj.qdot = new_qdot;

end

