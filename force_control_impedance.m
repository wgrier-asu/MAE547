%% Force Control 


%% Impedance Control
function [y] = impedance_control(x_d, xd_d, xdd_d, x_e, xd_e, he, P, D)
    %hd_e : end-effector forces w.r.t. desired frame

    hd_e = he; % wrong


    % y = pinv(J_Ad)*pinv(M_d)*(D*(xd_d-xd_e) + P*(xd-xe) - M_d*Jd_Ad*qd+M_d*bd-hd_e)
    % M_d*(xdd_d - y) + D*(xd_d-xd_e) + P*(x_d-x_e) = hd_e
    y = xdd_d - pinv(M_d)*(hd_e - D*(xd_d-xd_e) - P*(x_d-x_e))
    
end

%% Inverse Dynamics
% y : joint accelerations
% he : end-effector generalized forces
% q : joint positions
% qd : joint velocities
function [u] = invdyn(R, y, q, qd, he, g)
    qdd = y;
    grav = g;
    fext = he;
    u = R.rne(q, qd, qdd, grav, fext); % compute inverse dynamics
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