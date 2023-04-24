clc; close all; clear;

controlsbag = rosbag('robot_state_2023-04-11-16-42-01.bag');

rosbag info robot_state_2023-04-11-16-42-01.bag;

bSel = select(controlsbag,'Topic','/gvrbot_mobility_data');
msgStructs = readMessages(bSel,'DataFormat','struct');

msgStructs{1}

%Width of GVR bot is 40.5 cm. Assume each wheel is 10 cm. This means
%Width is approximately 30.5 cm

width = .305;

%Assume speed is in m/s


left_track_vel = cellfun(@(m) double(m.LeftTrackVelocity),msgStructs);
right_track_vel = cellfun(@(m) double(m.RightTrackVelocity),msgStructs);

x_points = zeros(1,length(left_track_vel) + 1);
y_points = zeros(1,length(right_track_vel) + 1);
x_old = x_points(1);
y_old = y_points(1);
theta_old = 0;

timestep = 1/15;

for i = 1:length(left_track_vel)
    velocity_new = left_track_vel(i) - right_track_vel(i) * timestep / 2;
    theta_new = theta_old + -1 * (( left_track_vel(i) + right_track_vel(i) ) * timestep / width ) / 2;
    x_new = x_old + cos(theta_new) * velocity_new * timestep;
    y_new = y_old + sin(theta_new) * velocity_new * timestep;
    x_points(i+1) = x_new;
    y_points(i+1) = y_new;
    theta_old = theta_new;
end

plot(x_points,y_points);
axis equal
title("Track Odometry")


%Velocity_right 
%Velocity_left 