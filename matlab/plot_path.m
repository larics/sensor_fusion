clc
clear

x = dlmread('ekf_x.mat');
y = dlmread('ekf_y.mat');
z = dlmread('ekf_z.mat');

x_truth = dlmread('truth_x.mat');
y_truth = dlmread('truth_y.mat');
z_truth = dlmread('truth_z.mat');

x_sensor = dlmread('sensor_front_x.mat');
y_sensor = dlmread('sensor_front_y.mat');
z_sensor = dlmread('sensor_front_z.mat');
% 
% x_model = dlmread('model_x.mat');
% y_model = dlmread('model_y.mat');
% z_model = dlmread('model_z.mat');
% 
x_sensor2 = dlmread('sensor_back_x.mat');
y_sensor2 = dlmread('sensor_back_y.mat');
z_sensor2 = dlmread('sensor_back_z.mat');

figure;
plot3(x,y,z,'LineWidth',2);
hold on;
grid on;
title("Path")
xlabel("X[m]");
ylabel("Y[m]");
zlabel("Z[m]");
plot3(x_truth,y_truth,z_truth,'LineWidth',2);
plot3(x_sensor,y_sensor,z_sensor,'--');
plot3(x_sensor2,y_sensor2,z_sensor2,'--k');
%plot3(x_model,y_model,z_model);
legend("Ekf","Istina","Sensor","Sensor2");
% 
size = size(x);
figure
hold on;
grid on;
title("Kvadratna greska")
xlabel("step")
xlim([100,250])
ylabel("kvadratno odstupanje")
plot(1:size(2),sqrt((x-x_truth).^2),'LineWidth',2.5);
plot(1:size(2),sqrt((x_sensor-x_truth).^2),'r','LineWidth',1.5);
plot(1:size(2),sqrt((x_sensor2-x_truth).^2));
legend("Ekf","Sensor","Sensor2")

% figure(3)
% grid on;
% hold on;
% plot(1:size(2),x_model)
% plot(1:size(2),y_model)
% plot(1:size(2),z_model)
% legend("x","y","z")
error_ekf = [sum((x-x_truth).^2)/size(2) sum((y-y_truth).^2)/size(2),sum((z-z_truth).^2)/size(2)]
error_sensor = [sum((x_sensor-x_truth).^2)/size(2), sum((y_sensor-y_truth).^2)/size(2), sum((z_sensor-z_truth).^2)/size(2)]



