% =========
% ass3_q1.m
% =========
%
% This assignment will introduce you to the idea of first building an
% occupancy grid then using that grid to estimate a robot's motion using a
% particle filter.
% 
% There are two questions to complete (5 marks each):
%
%    Question 1: code occupancy mapping algorithm 
%    Question 2: see ass3_q2.m
%
% Fill in the required sections of this script with your code, run it to
% generate the requested plot/movie, then paste the plots into a short report
% that includes a few comments about what you've observed.  Append your
% version of this script to the report.  Hand in the report as a PDF file
% and the two resulting AVI files from Questions 1 and 2.
%
% requires: basic Matlab, 'gazebo.mat'
%
% T D Barfoot, January 2016
%
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
vid = VideoWriter('ass3_q1.avi');
open(vid);
figure(1);
clf;
pcolor(ogp);
colormap(1-gray);
shading('flat');
axis equal;
axis off;
M = getframe(gcf);
writeVideo(vid,M);

% loop over laser scans (every fifth)
for i=1:5:size(t_laser,1)
    
    % ------insert your occupancy grid mapping algorithm here------

    % Find robot pose in pixels
    x_pix = floor((x_interp(i)-ogxmin)/ogres);
    y_pix = floor((y_interp(i)-ogymin)/ogres);
    
    % Loop over all laser readings
    for n = 1:npoints
        
        % Check to see if laser reading is valid
        if isnan(y_laser(i,n))
            continue
        end
        
        % Get indices of pixels in ray path:
        
        % First, get total angle of robot + laser
        phi = theta_interp(i) + angles(n);
        
        % Calculate laser range in pixels
        y_laser_px = y_laser(i,n)/ogres;
        
        % Get cell indices within laser range
        % Closer to y axis (ensure doesn't exceed y bounds)
        if abs(sin(phi)) > abs(cos(phi))
            
            % Get y value in world frame
            y_px = ceil(y_laser_px*sin(phi));
            
            % Check if outside map bounds
            if y_px + y_pix > ogny
                y_px = ogny-y_pix;
            end
            if y_px + y_pix < 0
                y_px = 0 - y_pix;
            end
            
            % Get list of y indices
            y_idx = 1:abs(y_px);

            if y_px < 0
                y_idx = -1*y_idx;
            end
            
            % Get list of x indixes
            x_idx = round(y_idx/tan(phi));
            
        % Closer to x axis (ensure doesn't exceed x bounds)
        else
            % Get x value in world frame
            x_px = ceil(y_laser_px*cos(phi));

            % Check if outside map bounds
            if x_px + x_pix > ognx
                x_px = ognx-x_pix;
            end
            if x_px + x_pix < 0
                x_px = 0 - x_pix;
            end
            
            % Get list of x indices
            x_idx = 1:abs(x_px);

            if x_px < 0
                x_idx = -1*x_idx;
            end
            
            % Get list of y indices
            y_idx = round(x_idx*tan(phi));
        end
        
        % Add robot cell pose values
        x_idx = x_idx + x_pix;
        y_idx = y_idx + y_pix;
        
        % For each cell in ray path
        for j=1:size(x_idx, 2)
            
            % Update log-odd occupancy grid
            if j < (size(x_idx,2))
                % Beta update (cell not in range)
                oglo(y_idx(j),x_idx(j)) = oglo(y_idx(j),x_idx(j)) - 1;
            else
                % Alpha update (cell in range)
                oglo(y_idx(j),x_idx(j)) = oglo(y_idx(j),x_idx(j)) + 1;
            end
                
        end
        
    end
    
    % Update probability occupancy grid
    ogp = exp(oglo)./(1.+exp(oglo));
    
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
    M = getframe(gcf);
    writeVideo(vid,M);
    
    pause(0.1);
    
end

close(vid);
print -dpng ass3_q1.png

save occmap.mat ogres ogxmin ogxmax ogymin ogymax ognx ogny oglo ogp;

