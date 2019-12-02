clear all;

% set random seed for repeatability
rng(1);

% ==========================
% load the dataset from file
% ==========================
%
%    ground truth poses: t_true x_true y_true theta_true
% odometry measurements: t_odom v_odom omega_odom
%           laser scans: t_laser y_laser
%    laser range limits: r_min_laser r_max_laser
%    laser angle limits: phi_min_laser phi_max_laser
%
load gazebo.mat;

% =======================================
% Question 1: build an occupancy grid map
% =======================================
%
% Write an occupancy grid mapping algorithm that builds the map from the
% perfect ground-truth localization.  Some of the setup is done for you
% below.  The resulting map should look like "ass2_q1_soln.png".  You can
% watch the movie "ass2_q1_soln.mp4" to see what the entire mapping process
% should look like.  At the end you will save your occupancy grid map to
% the file "occmap.mat" for use in Question 2 of this assignment.

% allocate a big 2D array for the occupancy grid
ogres = 0.05;                   % resolution of occ grid
ogxmin = -7;                    % minimum x value
ogxmax = 8;                     % maximum x value
ogymin = -3;                    % minimum y value
ogymax = 6;                     % maximum y value
ognx = (ogxmax-ogxmin)/ogres;   % number of cells in x direction
ogny = (ogymax-ogymin)/ogres;   % number of cells in y direction
oglo = zeros(ogny,ognx);        % occupancy grid in log-odds format
ogp = zeros(ogny,ognx);         % occupancy grid in probability format

% precalculate some quantities
numodom = size(t_odom,1);
npoints = size(y_laser,2);
angles = linspace(phi_min_laser, phi_max_laser,npoints);
dx = ogres*cos(angles);
dy = ogres*sin(angles);

% interpolate the noise-free ground-truth at the laser timestamps
t_interp = linspace(t_true(1),t_true(numodom),numodom);
x_interp = interp1(t_interp,x_true,t_laser);
y_interp = interp1(t_interp,y_true,t_laser);
theta_interp = interp1(t_interp,theta_true,t_laser);
omega_interp = interp1(t_interp,omega_odom,t_laser);
  
% set up the plotting/movie recording
vid = VideoWriter('ass2_q1.avi');
open(vid);
figure(1);
clf;
pcolor(ogp);
colormap(1-gray);
shading('flat');
axis equal;
axis off;
M = getframe;
writeVideo(vid,M);

a = theta_interp;  % redefine variable
ogp = repmat(0.5,ogny,ognx); % initialize probabilities to 0.5
% heuristic parameters
alpha = 1; % log_prob of occupied cell
beta = 0.5; % log_prob of free cell
% loop over laser scans (every fifth)
for i = 1:5:size(t_laser,1)
    % ------insert your occupancy grid mapping algorithm here------ 
    if abs(omega_interp(i)) < 0.1
        % rotation matrix to transform from vehicle to inertial frame
        H = [cos(a(i)) -sin(a(i));
            sin(a(i)) cos(a(i))];
        x_curr = [x_interp(i);y_interp(i)] - [ogxmin;ogymin];
        for j = 1:size(y_laser,2) 
            if ~isnan(y_laser(i,j)) && y_laser(i,j)<r_max_laser && y_laser(i,j)>r_min_laser
                % breaks down laser scan into segments. Incrementally go
                % through each segment and assume each measurement
                % corresponds to a unique cell.
                laser_step = r_min_laser:ogres:y_laser(i,j);
                for k = 1:length(laser_step)
                    % laser frame --> vehicle frame --> inertial frame
                    % note laser scans is 10 cm behind origin of robot
                    point = x_curr + H*laser_step(k)*[cos(angles(j));sin(angles(j))] - 0.1*[cos(a(i));sin(a(i))];
                    % round to nearest cell
                    grid_point = ceil(point/ogres);
                    x = grid_point(1);
                    y = grid_point(2);
                    if ~isnan(x) && ~isnan(y)
                        % if cell in FOV and near measured range (occupied)
                        if k==length(laser_step) 
                            oglo(y,x) = oglo(y,x) + alpha;
                            % if cell in FOV and closer than measured range (free)
                        elseif k<length(laser_step) 
                            oglo(y,x) = oglo(y,x) - beta;
                        end
                    end
                end
            end
        end
        % compute likelihood and thresholding based on prior (ogp)
        ogp_posterior = exp(oglo)./(1+exp(oglo));
        for y = 1:size(ogp, 1)
            for x = 1:size(ogp, 2)
                % if it's more likely that the cell is an obstacle
                if ogp(y,x)<ogp_posterior(y,x) && ogp_posterior(y,x)>=0.5
                    ogp(y,x) = ogp_posterior(y,x);
                % more likely than prior to be free cell
                elseif ogp(y,x)>ogp_posterior(y,x) && ogp_posterior(y,x)<0.5
                    ogp(y,x) = ogp_posterior(y,x);
                end
            end
        end

        % ------end of your occupancy grid mapping algorithm-------

        % draw the map
        clf;
        pcolor(ogp);
        colormap(1-gray);
        shading('flat');
        axis equal;
        axis off;

        % draw the robot
        hold on;
        x = (x_interp(i)-ogxmin)/ogres;
        y = (y_interp(i)-ogymin)/ogres;
        th = theta_interp(i);
        r = 0.15/ogres;
        set(rectangle( 'Position', [x-r y-r 2*r 2*r], 'Curvature', [1 1]),'LineWidth',2,'FaceColor',[0.35 0.35 0.75]);
        set(plot([x x+r*cos(th)]', [y y+r*sin(th)]', 'k-'),'LineWidth',2);

        % save the video frame
        M = getframe;
        writeVideo(vid,M);

        pause(0.1);
    end
    
end

% close(vid);
print -dpng ass2_q1.png

save occmap.mat ogres ogxmin ogxmax ogymin ogymax ognx ogny oglo ogp;

