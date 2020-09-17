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
x_dot = 0;
state = base_truth{5}.Pose.Pose.Position.X;
i = 1;
j = 1;
counter = 0;
%%%% for plot
State = [state];
base_state = [state];

for k = 5:size(base_truth)-10
    
    
    
    while 1
        
        if(speed{i}.Header.Stamp.Sec > base_truth{k}.Header.Stamp.Sec)
            break
        end
        if(speed{i}.Header.Stamp.Sec == base_truth{k}.Header.Stamp.Sec)
            if(speed{i}.Header.Stamp.Nsec > base_truth{k}.Header.Stamp.Nsec)
                break
            end
        end
        i = i + 1;
    end
    
    
    while 1
         if(imu{j}.Header.Stamp.Sec > base_truth{k}.Header.Stamp.Sec)
             break
         end
        if(imu{j}.Header.Stamp.Sec == base_truth{k}.Header.Stamp.Sec)
            if(imu{j}.Header.Stamp.Nsec > base_truth{k}.Header.Stamp.Nsec)
                break
            end
        end
        j = j + 1;
    end
    
    
%     disp([base_truth{k}.Header.Stamp.Sec ...
%     imu{j-1}.Header.Stamp.Sec...
%     speed{i-1}.Header.Stamp.Sec;...
%     base_truth{k}.Header.Stamp.Nsec...
%     imu{j-1}.Header.Stamp.Nsec...
%     speed{i-1}.Header.Stamp.Nsec])
    
    
    quat = [imu{j-1}.Orientation.W imu{j-1}.Orientation.X ...
        imu{j-1}.Orientation.Y imu{j-1}.Orientation.Z];
    angle = quat2eul(quat,'XYZ');
    
    psi = angle(3);
    theta = angle(2);
    fi = angle(1);
    
    x_ddot = (sin(psi)*sin(fi)+cos(psi)*sin(theta)*cos(fi))*...
            x*(speed{i-1}.AngularVelocities(1)^2+...
            speed{i-1}.AngularVelocities(2)^2+...
            speed{i-1}.AngularVelocities(3)^2+...
            speed{i-1}.AngularVelocities(4)^2);
    %x = b*k^2/m
    
    if (base_truth{k}.Header.Stamp.Sec == base_truth{k-1}.Header.Stamp.Sec)
        time = base_truth{k}.Header.Stamp.Nsec...
            - base_truth{k-1}.Header.Stamp.Nsec;
    else
        time = 1000000000 - base_truth{k-1}.Header.Stamp.Nsec ...
            + base_truth{k}.Header.Stamp.Nsec;
    end
   
    time = double(time)/1000000000;
    x_dot = x_dot + time*x_ddot;
    state = state + time*x_dot;% + 0.5*time^2*x_ddot;
    
    f = f + ...
       abs(state -...
        base_truth{k}.Pose.Pose.Position.X);
    
    State = [State, state];
    base_state = [base_state,...
        base_truth{k}.Pose.Pose.Position.X]; 
    counter = counter + 1;
end

f = f/(k-15);

% figure(3)
% plot(State)
% hold on
% plot(base_state)
% legend("state","base")