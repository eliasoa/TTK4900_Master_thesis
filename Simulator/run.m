close all
clear
%% Folder magic
% Get the current script's directory
currentFolder           = fileparts(which("TEST_FUNCTIONS.m"));

% Add the functions folder to the MATLAB path
addpath(fullfile(currentFolder, 'Functions'));

%% Geometric Information
% Size of MP
l_MP                = 0.15;
h_MP                = 0.02;
% Size of frame     
l_F                 = 1;
h_F                 = 1; 
[a, b, Wp, mp, Iz]   = initRobot(l_MP,h_MP,l_F,h_F);

%% Initial conditions
r                   = [  0.20; 
                         0.20];     % position of MP
phi                 = deg2rad(0);   % degrees

y0                  = [r;phi];      % pose
v0                  = zeros(3,1);   % velocity
[~,l0,~]            = InverseKinematics(a, b, y0); % cable length

initialCondition    = [y0;v0;l0'];

%% Force limits
f_min = 10;
f_max = 100;
level = 2;

%% Sølve sine ting
ts_w = planner_Solve(mp,Iz,initialCondition(1:6),a,b,f_min,f_max);

%% Reference
x_d                 = [ 0.2;
                        -0.2;
                        deg2rad(0)  ];
x_dd                = [ 0;
                        0;
                        0 ];
reference           = [x_d; x_dd];

%% Run simulink
simtime             = 10;    % How long to simulate
out                 = sim("cable_rob.slx");

%% Fra OM
time = out.simout.Time;
data = out.simout.Data;

p = data(:,1:2);
psi = data(:,3);

% ball_t = out.ball_p.Time;
% ball_p = out.ball_p.Data;

% Anchor points
a1 = a(:,1);
a2 = a(:,2);
a3 = a(:,3);
a4 = a(:,4);

% Body anchors
b1 = b(:,1);
b2 = b(:,2);
b3 = b(:,3);
b4 = b(:,4);

axes_limits = [-.5,.5,-.5,.5];
for n = 1:max(size(time))
    % Body anchors in global coordinates
    b1g = b_to_g(p(n,:)',psi(n),b1);
    b2g = b_to_g(p(n,:)',psi(n),b2);
    b3g = b_to_g(p(n,:)',psi(n),b3);
    b4g = b_to_g(p(n,:)',psi(n),b4);
    sq = [b1g,b2g,b3g,b4g,b1g];
    h=plot(sq(1,:),sq(2,:),'b');
    hold on;
    plot([a1(1),b1g(1)],[a1(2),b1g(2)],'r');
    plot([a2(1),b2g(1)],[a2(2),b2g(2)],'r');
    plot([a3(1),b3g(1)],[a3(2),b3g(2)],'r');
    plot([a4(1),b4g(1)],[a4(2),b4g(2)],'r');
    % plot(ball_p(n,1),ball_p(n,2),'*');
    hold off;
    axis(axes_limits);
    set(h,"LineWidth",1);
    % axis equal;
    grid;
    
    % Fra ChatGPT for å vise simuleringstid
    % Calculate midpoint for x-coordinate of the text
    mid_x = mean(axes_limits(1:2));
    % Set y-coordinate for the text slightly above the top of the plot
    text_y = axes_limits(4) + 0.05;

    % Displaying the simulation time
    text(mid_x, text_y, sprintf('Time: %.2f seconds', time(n)), ...
         'HorizontalAlignment', 'center', 'VerticalAlignment', 'bottom');

    if n==1
        pause;
    else
        pause(0.001);
    end
end
