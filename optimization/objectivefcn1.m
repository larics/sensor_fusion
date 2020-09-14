function f = objectivefcn1(x)

bag = rosbag('2020-09-13-16-49-58.bag');
bSel = select(bag,'Topic','/uav/odometry');
base_truth = readMessages(bSel,'DataFormat','struct');


bSel = select(bag,'Topic','/uav/motor_speed');
speed = readMessages(bSel,'DataFormat','struct');

bSel = select(bag,'Topic','/uav/imu');
imu = readMessages(bSel,'DataFormat','struct');

f = 0;
size(speed);
size(base_truth);

for k = 1:size(base_truth)-10
    
    quat = [imu{5*k+168}.Orientation.W imu{5*k+168}.Orientation.X ...
        imu{5*k+168}.Orientation.Y imu{5*k+168}.Orientation.Z];
    angle = quat2eul(quat,'XYZ');
    
    psi = angle(3);
    theta = angle(2);
    fi = angle(1);
    
    f = f + ...
       ((sin(psi)*sin(fi)+cos(psi)*sin(theta)*cos(fi))*...
            x(1)*(speed{k*5+64}.AngularVelocities(1)^2+...
            speed{k*5+64}.AngularVelocities(2)^2+...
            speed{k*5+64}.AngularVelocities(3)^2+...
            speed{k*5+64}.AngularVelocities(4)^2)/x(2)-...
            base_truth{k}.Pose.Pose.Position.X)^2;
end