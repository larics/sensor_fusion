function f = objectivefcn1(x)

% bag = rosbag('2020-09-13-16-49-58.bag');
% bSel = select(bag,'Topic','/uav/odometry');
% base_truth = readMessages(bSel,'DataFormat','struct');
% 
% 
% bSel = select(bag,'Topic','/uav/motor_speed');
% speed = readMessages(bSel,'DataFormat','struct');
% 
% bSel = select(bag,'Topic','/uav/imu');
% imu = readMessages(bSel,'DataFormat','struct');

% bSel = select(bag,'Topic','/uav/pid_roll');
% roll = readMessages(bSel,'DataFormat','struct');
% 
% bSel = select(bag,'Topic','/uav/pid_pitch');
% pitch = readMessages(bSel,'DataFormat','struct');
% 
% bSel = select(bag,'Topic','/uav/pid_yaw');
% yaw = readMessages(bSel,'DataFormat','struct');

bag = rosbag('optimization_bag_2020-09-14-11-27-17.bag');
bSel = select(bag,'Topic','/blue/mavros/global_position/local');
base_truth = readMessages(bSel,'DataFormat','struct');


bSel = select(bag,'Topic','/blue/mavros/rc/out');
speed = readMessages(bSel,'DataFormat','struct');

bSel = select(bag,'Topic','/blue/mavros/imu/data');
imu = readMessages(bSel,'DataFormat','struct');

f = 0;
Phi = [];Theta = []; Psi = [];
Roll = []; Pitch = []; Yaw = [];
x_dot = x(2);
state = base_truth{10}.Pose.Pose.Position.X;
i = 1;
j = 1;
counter = 0;
acc = [0,0,0,0,0];
%%%% for plot
State = [state];
base_state = [state];


% for m = 1:size(yaw)
%     Roll = [Roll, roll{m}.Meas];
%     Pitch = [Pitch, pitch{m}.Meas];
%     Yaw = [Yaw, yaw{m}.Meas];
% end

for iter = 1:15
    start = 150*(iter-1)+5;
    endo = 150*iter+5;
    x_dot = x_dot;
    state = state;
    
    for k = start:endo
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
    %%%%5 yaw pitch roll
    angle = quat2eul(quat);
    
    %%%%provjeri
    psi = angle(1);
    theta = angle(2);
    fi = angle(3);
    Phi = [Phi, fi]; Psi = [Psi, psi]; Theta = [Theta, theta];  
    x_ddot = (sin(psi)*sin(fi)+cos(psi)*sin(theta)*cos(fi))...%*(imu{j-1}.LinearAcceleration.Z)
            *x(1)*(...
            double(speed{i-1}.Channels(1)^2)+...
            double(speed{i-1}.Channels(2)^2)+...
            double(speed{i-1}.Channels(3)^2)+...
            double(speed{i-1}.Channels(4)^2));
    %(imu{j-1}.LinearAcceleration.Z);
    %x = b*k^2/m
    
    acc = [acc; x_ddot,imu{j-1}.LinearAcceleration.X,fi,psi,theta];
    
    
    if (base_truth{k}.Header.Stamp.Sec == base_truth{k-1}.Header.Stamp.Sec)
        time = base_truth{k}.Header.Stamp.Nsec...
            - base_truth{k-1}.Header.Stamp.Nsec;
    else
        time = 1000000000 - base_truth{k-1}.Header.Stamp.Nsec ...
            + base_truth{k}.Header.Stamp.Nsec;
    end
   
    time = double(time)/1000000000;
    x_dot = x_dot + time*x_ddot;
    state = state + time*x_dot + 0.5*time^2*x_ddot;
    
    f = f + ...
       abs(state -...
        base_truth{k}.Pose.Pose.Position.X);

%       f = f + abs(x_ddot+imu{j-1}.LinearAcceleration.X);
    
    State = [State, state];
    base_state = [base_state,...
        base_truth{k}.Pose.Pose.Position.X]; 
    counter = counter + 1;
    end
end
f = f/counter;
% disp(x);
% figure(3)
% plot(State)
% hold on
% plot(base_state)
% legend("state","base")

% figure(1)
% hold on
% subplot(3,1,1)
% plot(Psi)

f = acc;

% legend("PSI","yaw")
% subplot(3,1,2)
% plot(Theta)
% 
% legend("theta","pitch")
% subplot(3,1,3)
% plot(Phi)
% 
% legend("Phi","roll")
% 
% figure(4)
% subplot(3,1,1)
% plot(Yaw)
% subplot(3,1,2)
% plot(Pitch)
% subplot(3,1,3)
% plot(Roll)