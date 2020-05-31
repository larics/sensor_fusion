clc
clear
close all

x = dlmread('ekf_x.mat'); 
y = dlmread('ekf_y.mat');
z = dlmread('ekf_z.mat');

x_truth = dlmread('truth_x.mat');
y_truth = dlmread('truth_y.mat');
z_truth = dlmread('truth_z.mat');

x_sensor = dlmread('sensor_front_x.mat');
y_sensor = dlmread('sensor_front_y.mat');
z_sensor = dlmread('sensor_front_z.mat');

x_sensor2 = dlmread('sensor_back_x.mat');
y_sensor2 = dlmread('sensor_back_y.mat');
z_sensor2 = dlmread('sensor_back_z.mat');
% 
x_sensor3 = dlmread('sensor_mid_x.mat');
y_sensor3 = dlmread('sensor_mid_y.mat');
z_sensor3 = dlmread('sensor_mid_z.mat');
a = "test"
cd 'Photos_3 senzora'/
figure;
title("Putanja letjelice")
plot3(x,y,z,'LineWidth',2);
hold on;
grid on;
title("Path")
xlabel("X $[m]$",'Interpreter','latex');
ylabel("Y $[m]$",'Interpreter','latex');
zlabel("Z $[m]$",'Interpreter','latex');
plot3(x_truth,y_truth,z_truth,'LineWidth',2);
plot3(x_sensor,y_sensor,z_sensor,'--');
plot3(x_sensor2,y_sensor2,z_sensor2,'--k');
plot3(x_sensor3,y_sensor3,z_sensor3,'--');
legend("Ekf","Ground truth","Senzor 1","Senzor 2","Senzor 3",'Location','best');
print("Putanja_letjelice3d"+a,'-dpng')

figure;
title("Putanja letjelice")
plot(x,y,'LineWidth',2);
hold on;
grid on;
title("Path")
xlabel("X $[m]$",'Interpreter','latex');
ylabel("Y $[m]$",'Interpreter','latex');
plot(x_truth,y_truth,'LineWidth',2);
plot(x_sensor,y_sensor,'--');
plot(x_sensor2,y_sensor2,'--k');
plot(x_sensor3,y_sensor3,'--');
legend("Ekf","Ground truth","Senzor 1","Senzor 2","Senzor 3",'Location','best');
print("Putanja_letjelice2d"+a,'-dpng')
% % 
 size = size(x);
figure
hold on;
grid on;
title("Odstupanje od realne vrijednosti, X-os")
xlabel("Korak")
% xlim([100,250])
ylabel("Odstupanje $[m]$",'Interpreter','latex')
plot(1:size(2),sqrt((x-x_truth).^2),'LineWidth',3);
plot(1:size(2),sqrt((x_sensor-x_truth).^2));
plot(1:size(2),sqrt((x_sensor2-x_truth).^2));
legend("Ekf","Senzor 1","Senzor 2")
print("Odstupanje_X"+a,'-dpng')

figure
hold on;
grid on;
title("Odstupanje od realne vrijednosti, Y-os")
xlabel("Korak")
% xlim([100,250])
ylabel("Odstupanje $[m]$",'Interpreter','latex')
plot(1:size(2),sqrt((y-y_truth).^2),'LineWidth',3);
plot(1:size(2),sqrt((y_sensor-y_truth).^2));
plot(1:size(2),sqrt((y_sensor2-y_truth).^2));
legend("Ekf","Senzor 1","Senzor 2")
print("Odstupanje_y"+a,'-dpng')

figure
hold on;
grid on;
title("Odstupanje od realne vrijednosti, Z-os")
xlabel("Korak")
% xlim([100,250])
ylabel("Odstupanje $[m]$",'Interpreter','latex')
plot(1:size(2),sqrt((z-z_truth).^2),'LineWidth',3);
plot(1:size(2),sqrt((z_sensor-z_truth).^2));
plot(1:size(2),sqrt((z_sensor2-z_truth).^2));
legend("Ekf","Senzor 1","Senzor 2")
print("Odstupanje_z"+a,'-dpng')


figure;

subplot(3,1,1)
title("Trajektorija letjelice")
grid on;
hold on;
xlabel("Korak")
ylabel("X $[m]$",'Interpreter','latex')
plot(1:size(2),x,'LineWidth',2)
plot(1:size(2),x_truth)
plot(1:size(2),x_sensor)
plot(1:size(2),x_sensor2)
%legend("Ekf","Istina","Senzor 1","Senzor 2",'Location','best');
subplot(3,1,2)
grid on;
hold on;
xlabel("Korak")
ylabel("Y $[m]$",'Interpreter','latex')
plot(1:size(2),y,'LineWidth',2)
plot(1:size(2),y_truth)
plot(1:size(2),y_sensor)
plot(1:size(2),y_sensor2)
subplot(3,1,3)
grid on;
hold on;
xlabel("Korak")
ylabel("Z $[m]$",'Interpreter','latex')
plot(1:size(2),z,'LineWidth',2)
plot(1:size(2),z_truth)
plot(1:size(2),z_sensor)
plot(1:size(2),z_sensor2)
legend("Ekf","Istina","Senzor 1","Senzor 2",'Location','best');

print("Trajektorija"+a,'-dpng')

error_ekf = sqrt([sum((x-x_truth).^2)/size(2) sum((y-y_truth).^2)/size(2),sum((z-z_truth).^2)/size(2)])'
error_sensor = sqrt([sum((x_sensor-x_truth).^2)/size(2), sum((y_sensor-y_truth).^2)/size(2), sum((z_sensor-z_truth).^2)/size(2)])'
error_sensor2 = sqrt([sum((x_sensor2-x_truth).^2)/size(2), sum((y_sensor2-y_truth).^2)/size(2), sum((z_sensor2-z_truth).^2)/size(2)])'

figure;
title("Odstupanje X-osi od stvarne vrijednosti");
histogram(x_sensor-x_truth,20);
hold on
xlabel("Odstupanje")
ylabel("n")
histogram(x-x_truth);
legend("Senzor","EKF")
title("Odstupanje X-osi od stvarne vrijednosti");
print("Histogram_X"+a,'-dpng')
figure;

histogram(y_sensor-y_truth,20);
hold on
xlabel("Odstupanje")
ylabel("n")
histogram(y-y_truth);
legend("Senzor","EKF")
title("Odstupanje Y-osi od stvarne vrijednosti");
print("Histogram_Y"+a,'-dpng')
figure;

histogram(z_sensor-z_truth,20);
title("Odstupanje Z-osi od stvarne vrijednosti");
hold on
xlabel("Odstupanje")
ylabel("n")
histogram(z-z_truth);
legend("Senzor","EKF")
print("Histogram_Z"+a,'-dpng')

cd ..