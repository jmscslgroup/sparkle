 % (C) RAHUL BHADANI
 % Reading a bag file

 %% Read the bag file
bagReader = rosbag('/home/ivory/VersionControl/catvehicle_ws/src/sparkle/2019-08-05-21-41-45.bag');
bagReader.AvailableTopics

%% 
odomSelect = select(bagReader, 'Topic', '/nebula/odom');
odom_poseX = timeseries(odomSelect, 'Pose.Pose.Position.X');
odom_poseY = timeseries(odomSelect, 'Pose.Pose.Position.Y');

% Orientations are in Quaternions
qx = timeseries(odomSelect, 'Pose.Pose.Orientation.X');
qy = timeseries(odomSelect, 'Pose.Pose.Orientation.Y');
qz = timeseries(odomSelect, 'Pose.Pose.Orientation.Z');
qw = timeseries(odomSelect, 'Pose.Pose.Orientation.W');


%%
[angles.roll, angles.pitch, angles.yaw] =  queternionToRPY(qx.Data, qy.Data, qz.Data, qw.Data);

rpySelect = select(bagReader, 'Topic', '/nebula/angles');
r = timeseries(rpySelect, 'X'); % Roll
p = timeseries(rpySelect, 'Y'); % Pitch
y = timeseries(rpySelect, 'Z'); % Yaw



% Plot now

fig= figure;
fig.Position = [799 334 850 494];
plot(y.Time, y.Data,'--','LineWidth',2);
hold on;
plot(qz.Time, angles.yaw,'-.','LineWidth',2);

xlabel('\textbf{Time}','Interpreter','latex');
ylabel({'\textbf{Angles}'},'Interpreter','latex');
set(gca,'FontSize',15);
set(gca,'FontName','Times');
title('\textbf{Yaw - Timeseries [;pt}','Interpreter','latex');
grid on;
grid minor;
L = legend({'\textbf{Simulink yaw}',...
    '\textbf{Gazebo yaw}'},'Interpreter','latex');
L.FontSize = 15;

function [roll, pitch, yaw]  = queternionToRPY(qx, qy, qz, qw)

    length(qx)
    length(qy)
    length(qz)
    length(qw)
    
    % roll (x-axis rotation)
    sinr_cosp = 2.0.*((qw .* qx) + (qy .* qz));
    cosr_cosp = 1  - 2* ( (qx .* qx) + (qy.*qy));
    roll = atan2(sinr_cosp, cosr_cosp);

    % pitch (y-axis rotation)
    sinp = 2.0 * ( (qw.* qy) - (qz .* qx));
    if (abs(sinp) >= 1)
        pitch = (pi/2)*sign(sinp); % use 90 degrees if out of range
    else
        pitch = asin(sinp);
    end

    % yaw (z-axis rotation)
    siny_cosp = 2.0*((qw .* qz) + (qx .* qy));
    cosy_cosp = 1.0 - 2.0*((qy .* qy) + (qz .* qz));  
    yaw = atan2(siny_cosp, cosy_cosp);

end


