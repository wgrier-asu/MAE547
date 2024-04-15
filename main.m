close all
mypath = fileparts( mfilename('fullpath') );
rvcpath = fullfile(rvcpath, 'rvctools');
if exist(rvcpath,'dir')
    addpath(rvcpath);
    startup_rvc
end

%% BEGIN ROBOT SPECIFICATION %
% DH Parameter Inputs
a = [4 3 2 2];
d = [0 0 0 0];
alpha = [0 0 0 0];
theta = [0 0 0 pi/2];
DH = [d' theta' a' alpha'];

% Robot Options
OPTIONS.numLinks    = 4;
OPTIONS.workspace   = max(a)*3;     % Workspace Size
OPTIONS.gravity     = [0 0 9.81];   % Gravity Vector
% Link Options
qlims   = cell(OPTIONS.numLinks, 1);
I       = cell(OPTIONS.numLinks, 1);
COM     = cell(OPTIONS.numLinks, 1);
TC      = cell(OPTIONS.numLinks, 1);
for i=1:OPTIONS.numLinks
    qlims{i} = [-3 3];
    I{i} = eye(3);
    COM{i} = zeros(3, 1);
    TC{i} = zeros(2, 1);
end

OPTIONS.isRevolute  = [1 0 1 0];                % Link Type (1=revolute, 0=prismatic)
OPTIONS.offset      = zeros(OPTIONS.numLinks);  % Joint Param. Offset
OPTIONS.qlims       = qlims;                    % Joint Param. Limits
OPTIONS.inertia     = I;                        % Inertia Matrix
OPTIONS.mass        = zeros(OPTIONS.numLinks);  % Mass
OPTIONS.centerOfMass= COM;                      % Center of Mass
OPTIONS.jointFric   = zeros(OPTIONS.numLinks);  % Joint Friction
OPTIONS.coulombFric = TC;                       % Coulomb/Static Friction
OPTIONS.gearRatio   = ones(OPTIONS.numLinks);   % Gear Ratio
OPTIONS.motorInertia= zeros(OPTIONS.numLinks);  % Motor Inertia

% END ROBOT SPECIFICATION %

% Create Robot
robot = make_robot(DH, OPTIONS);
robot.dyn
robot.teach()




% Desired Pose Inputs
P = [-9; 0; 0]; % Desired Position w.r.t. Base Frame
R = [0 1 0; -1 0 0; 0 0 1]; % Desired Rotation w.r.t. Base Frame

%%
rtbdemo()


%%
% Inverse Kinematics %
function [q] = inv_solve(P, R, robot)
    % Validate Input
    pdims = size(P);
    if(pdims(1) ~= 3 || pdims(2) ~= 1)
        ME = MException('ikin:typeError', ...
            'Position vector P has incorrect format. Expected: 3x1 Vector');
        throw(ME)
    end
    rdims = size(R);
    if(rdims(1) ~= 3 || rdims(2) ~= 3)
        ME = MException('ikin:typeError', ...
            'Rotation matrix R has incorrect format. Expected: 3x3 Matrix');
        throw(ME)
    end


    % Format Homogenous Transformation
    R = [R; 0 0 0];
    P = [P; 1];
    T = [R, P]
    
    
    
    % Solve Inverse Kinematics
    q_corke = robot.ikunc(T)
    q = q_corke;
end


