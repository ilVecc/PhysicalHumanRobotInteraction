%% show robot
UR5_robot = importrobot('./ur_description_noEE/urdf/ur5.urdf');
UR5_robot.DataFormat = 'column';
%showdetails(UR5_robot);
%config = homeConfiguration(UR5_robot);
show(UR5_robot,conf.init.q0);
xlim([-0.4 0.4]);
ylim([-0.5 0.5]);
zlim([-0.1 0.6]);