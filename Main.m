clear
clc
close all


mu0 = 4*pi*1e-7; %Vs/AM = N/A^2
f = 2;
pi = 3.14;

%% Actuator Magnet properties - center at (0,0,0)

M0 = 4.17e6; % volume magnetization [A/m] (it is for the actuator magnet)
%l = 0.0127; %m actuator magnet size
%a = 0.0254; %m

a = 0.0127/2; % Radius of Cylinder, Unit m
l = 0.0127/4;  % Semi-Length of Cylinder, Unit m


om = 2*pi*f; %rotation speed in rad/s
Vy = 0.002; %translational speed along y in m/s



%% Robot Magnet Properties - center at (0,0,0.05)

vm = 1.927e-10; %volume of robot magnet
m0 = [0 ;0 ;4.17e6]; % this one is for the robot TO BE CHANGED
m1 = [0; 4.17e6 ;0]; % this one is for the robot TO BE CHANGED

loc = 0.05; %location of center of robot magnet
m = 0.01; % magnet mass in kg (measure the weight at the kab)



%% Counters

tend = 5; % length of time

zc = 0:0.001:loc; %discretization of the distance between the center of magnets

t = 0:0.001:tend; %discretization of time (s)

Nz = size(zc,2);
Nt = size(t,2);




%% Calculation of rotating and translating magnetic field

for j = 1:Nt
   for i=1:Nz

        % calculation of field along z
        term1 = (zc(i)+l/2)/(a^2+(zc(i)+l/2)^2)^0.5;
        term2 = (zc(i)-l/2)/(a^2+(zc(i)-l/2)^2)^0.5;
        Bc = mu0*(M0*l)*(term1-term2)/(2*l); % Tesla = N/Am
        B(i)= Bc; %Tesla
        Bg(i) = Bc *1e4; %Gauss
        Bry(j, i) = 0;
   end
   
   
    %Definition of rotating magnetic field
    Brx(j, :) = B*cos(om*j);
    Brz(j, :) = B*sin(om*j);
   
    
end



%% Plotting B at one time-step

figure;
plot(zc, B, 'LineWidth', 2);
title('Magnetic Field B (Tesla)');
xlabel('Distance along z (m)');
ylabel('Magnetic Field B (Tesla)');
grid on;

%% Plotting B at one location

figure;
scatter(t, Brx(:, Nz),4, 'filled');
hold on;
scatter(t, Brz(:, Nz),4, 'filled')
title('Rotation of Magnetic Field over Time');
xlabel('time (t)');
ylabel('Magnetic Field B (Tesla)');
legend({'B Cos(omega t)','B Sin(omega t)'});
grid on;

%% Plotting Brx and Brz over time at the last Nz in 3D

figure;
scatter3(t, Brx(:, Nz), Brz(:, Nz),4, 'filled');
title('Rotating Magnetic Field Components at the last Nz over time');
xlabel('Time (s)');
ylabel('Bx (Tesla)');
zlabel('Bz (Tesla)');
grid on;

%% Calculation of gradient


for i = 1:Nz
                 if i == 1
        dz(i) = (B(i+1) - B(i)) /0.001;
    elseif i == Nz
        dz(i) = (B(i) - B(i-1)) /0.001;
    else
        dz(i) = (B(i+1) - B(i-1)) /0.001;
                 end
end

% Plotting Gradient at one time-step

figure;
plot(zc, dz, 'LineWidth', 2);
title('Gradient Magnetic Field');
xlabel('Distance along z (m)');
ylabel('Magnetic Field Gradient');
grid on;



for j = 1:Nt
    
    
   for i=1:Nz
     
          if i == 1
        dBdx(j, i) = (Brx(j, i+1) - Brx(j, i)) /0.001;
    elseif i == Nz
        dBdx(j, i) = (Brx(j, i) - Brx(j, i-1)) /0.001;
    else
        dBdx(j, i) = (Brx(j, i+1) - Brx(j, i-1)) /0.001;
          end
          
         
                    if i == 1
        dBdy(j, i) = (Bry(j, i+1) - Bry(j, i)) /0.001;
    elseif i == Nz
        dBdy(j, i) = (Bry(j, i) - Bry(j, i-1)) /0.001;
    else
        dBdy(j, i) = (Bry(j, i+1) - Bry(j, i-1)) /0.001;
          end
       
       
          if i == 1
        dBdy(j, i) = (Bry(j, i+1) - Bry(j, i)) /0.001;
    elseif i == Nz
        dBdy(j, i) = (Bry(j, i) - Bry(j, i-1)) /0.001;
    else
        dBdy(j, i) = (Bry(j, i+1) - Bry(j, i-1)) /0.001;
          end
          
       
       if i == 1
        dBdz(j, i) = (Brz(j, i+1) - Brz(j, i)) /0.001;
    elseif i == Nz
        dBdz(j, i) = (Brz(j, i) - Brz(j, i-1)) /0.001;
    else
        dBdz(j, i) = (Brz(j, i+1) - Brz(j, i-1)) /0.001;
       end


   end

end



%% Calculation of Force or Torque - Bullshit from this point
  
initial_dBdx = dBdx(1, :);
initial_dBdy = dBdy(1, :);
initial_dBdz = dBdz(1, :);

initial_Brx = zeros(1,51); 
initial_Bry = zeros(1,51);
initial_Brz = B(1, :);

initial_grad =  [initial_dBdx; initial_dBdy; initial_dBdz];
initial_B = [initial_Brx; initial_Bry; initial_Brz];

F(:,:) = vm * (m0.* initial_grad);

for i = 1:Nz

    T(:, i) = (vm * cross(m1, [initial_Brx(i) initial_Bry(i) initial_Brz(i)]));
end


% Plot F and T

figure;
plot(zc, F(3, :), 'b', 'LineWidth', 2); % Plot F in blue
%title('Force');
xlabel('Distance along z (m)');
ylabel('Force (N)');
grid on;

figure;
plot(zc, T(1, :), 'b', 'LineWidth', 2); % Plot T in red
%title('Torque');
xlabel('Distance along z (m)');
ylabel('Torque (N.m)');
grid on;