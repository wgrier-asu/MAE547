%% Force Control 


%% Impedance Control
function [y] = impedance_force_control(R, o_d, R_d, od_d, Rd_d, odd_d, Rdd_d, o_e, R_e, od_e, Rd_e, he)
    % TODO
end

%% Inverse Dynamics
% y : joint accelerations
% he : end-effector generalized forces
% q : joint positions
% qd : joint velocities
function [u] = invdyn(R, y, q, qd, he)
    qdd = y;
    grav = (0 0 -9.8)';
    fext = he;
    u = R.rne(q, qd, qdd, grav, fext); % compute inverse dynamics
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

% TODO
function g = gravity(q)
    g = q*0;
end

% PD Controller with Gravity Compensation
function tau = mytorqfun(t, q, qd, R, xd, P, D)
    T = R.fkine(q);
    xe = T(1:3, 4);

    g = gravity(q);

    J = R.jacob0(q, 'eul');

    tau = g + J'*P*(xd-xe) - J'*D*J*qd; 
end

function [q,qd] = manipulator_model(torqfun)
    [t,q,qd] = R.fdyn(1, torqfun, R, xd, P, D);
end