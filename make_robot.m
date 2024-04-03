function [robot] = make_robot(DH, OPTIONS)
%MAKE_ROBOT Constructs a SerialLink Object
%   Provide the DH parameters and options for kinematics and dynamics
%   DH Input
%       DH = [d theta a alpha];
%       d, theta, a, alpha are nx1 arrays
%       DH is an nx4 matrix
% 
%   OPTIONS
%   see SerialLink documentation for details about input format
%
    % Validate Input
    dhdims = size(DH)';
    if(dhdims(2) ~= 4)
        ME = MException('ikin:typeError', ...
            'DH Parameters are in incorrect format. Expected nx5 Matrix');
        throw(ME)
    end

    numLinks = dhdims(1);
    fprintf('\nNumber of Links: %d', numLinks)

    % Create Links
    L = cell(numLinks, 1);
    for i = 1:numLinks
        % Collect Inputs
        dh = DH(i,:);
        O = OPTIONS.offset(i);
        qlim = OPTIONS.qlims{i};
        isRevolute = OPTIONS.isRevolute(i);
        I = OPTIONS.inertia{i};
        r = OPTIONS.centerOfMass{i};
        m = OPTIONS.mass(i);
        G = OPTIONS.gearRatio(i);
        B = OPTIONS.jointFric(i);
        Jm = OPTIONS.motorInertia(i);
        Tc = OPTIONS.coulombFric{i};

        % Create Link Object
        if(isRevolute)
%             fprintf('\n\tLink %d is Revolute', i);
            L{i} = Revolute('d', dh(1), 'a', dh(3), 'alpha', dh(4), 'offset', O, 'I', I, 'r', r, 'm', m, 'G', G, 'B', B, 'Jm', Jm, 'Tc', Tc);
        else
%             fprintf('\n\tLink %d is Prismatic', i);
            L{i} = Prismatic('theta', dh(2), 'a', dh(3), 'alpha', dh(4), 'offset', O, 'qlim', qlim, 'I', I, 'r', r, 'm', m, 'G', G, 'B', B, 'Jm', Jm, 'Tc', Tc);
        end
%         Display link dynamics and kinematics
%         L{i}.dyn
    end

    % Create Serial Robot
    R = SerialLink(horzcat(L{:}));

    % Create workspace
    SIZE = OPTIONS.workspace;
    R.name = 'robot';
    R.plotopt = {'workspace' [-SIZE,SIZE,-SIZE,SIZE,-SIZE,SIZE]};
    R.gravity = OPTIONS.gravity;

    robot = R;
end