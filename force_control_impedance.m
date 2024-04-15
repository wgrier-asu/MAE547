%% Force Control 


%% Inverse Dynamics
% y : joint accelerations
% he : end-effector generalized forces
% q : joint positions
% qd : joint velocities
function [u] = invdyn(R, y, q, qd, he)
    u = R.rne(q, qd, y); % compute inverse dynamics
end

%% Direct Kinematics
function [o, R, od, omega] = direct_kinematics(R, q, qd)
    T = R.fkine(q);
    o = T(1:3, 4);
    R = T(1:3, 1:3);

    J = R.jacob0(q);
    dX = J*qd;

    od = dX(1:3);
    omega = dX(4:6);

    % dQ/dt = J(q)^-1 dX/dt
end

%% Manipulator Dynamics
% Torque to [End-Effector Force, Joint Positions, Joint Velocities]
% u : torque
% he : end-effector generalized forces
% q : joint positions
% qd : joint velocities

% function [he, q, qd] = manipulator(u)
%     
% end