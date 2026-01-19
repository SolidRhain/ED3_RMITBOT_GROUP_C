clc; clear; close all;

% Inverse equation, take in desired motion, calculate wheel's angular
% velocity based on desired velocity Vx, Vy and omega.

% Expected velocity 
Vx = 0;        % Linear Velocity in x axis
Vy = 0;        % Linear Velocity in Y axis
w = 0;         % Rotating velocity 
theta = 35;    % Angle of desired motion 

% Robot parameters
r = 0.08;     % Wheel Radius
Lx = 0.1;  % Half Length
Ly = 0.1;    % Half Width
L = Lx+Ly; % Overall Size

V = 1;   % Linear velocity
Vxy = 1; % Diagnol velocity

fprintf('Select robot motion direction:\n');
fprintf('1. Forward\n');
fprintf('2. Backward\n');
fprintf('3. Strafe Left\n');
fprintf('4. Strafe Right\n');
fprintf('5. Front Left\n');
fprintf('6. Front Right\n');
fprintf('7. Rear Left\n');
fprintf('8. Rear Right\n');
fprintf('9. Rotate Left\n');
fprintf('10. Rotate Right\n');

Direction = input('Enter direction: ', 's');

switch Direction
    case "Forward"
        Vx = V; 
        Vy = 0;
        w = 0;
    case "Backward"
        Vx = -V;
        Vy = 0;
        w = 0;
    case "Strafe Left"
        Vx = 0;
        Vy = V;
        w = 0;
    case "Strafe Right"
        Vx = 0;
        Vy = -V;
        w = 0;
    case "Front Left"
        Vx = Vxy * cos(theta);
        Vy = Vxy * sin(theta);
        w = 0;
    case "Front Right"
        Vx = Vxy * cos(theta);
        Vy = Vxy * (- sin(theta));
        w = 0;
    case "Rear Left"
        Vx = Vxy * (- cos(theta));
        Vy = Vxy * (- sin(theta));
        w = 0;
    case "Rear Right"
        Vx = Vxy * (- cos(theta));
        Vy = Vxy * (- sin(theta));
        w = 0;
    case "Rotate Left"
        Vx = 0;
        Vy = 0;
        w = V;
    case "Rotate Right"
        Vx = 0;
        Vy = 0;
        w = -V;
end 

% Inverse Kinematics equation (From controller to wheel)
Inverse_equation = (1/r) * [1  -1  -L;
                            1   1   L;
                            1  -1   L;
                            1   1  -L];
Vel = [Vx; Vy; w];

Wi = Inverse_equation * Vel; % Wheel Velocity Calculation

w1 = Wi(1);
w2 = Wi(2);
w3 = Wi(3);
w4 = Wi(4);

% Print result
fprintf('\nMotion type: %s\n', Direction);
fprintf('Vx = %.3f m/s, Vy = %.3f m/s, w = %.3f rad/s\n', Vx, Vy, w);
fprintf('Wheel speed (rad/s):\n');
fprintf('wheel 1 = %.3f\nwheel 2 = %.3f\nwheel 3 = %.3f\nwheel 4 = %.3f\n', w1, w2, w3, w4);

%% 

% Forward Equation, receiving motor real speed, calculate acutal speed of robot

clc; clear; close all;

% Robot dimesion
r = 1; % Wheel radius
Lx = 1.5;  % Half Length
Ly = 1;    % Half Width
L = Lx+Ly; % Overall Size]

% Wheel Angular velocity in "Forward" motion
w1 = 5;
w2 = 5;
w3 = 5; 
w4 = 5;
wi = [w1; w2; w3; w4]; % Angular velocity of all 4 wheels

% Forward kinematic equation (From wheel to controller)
Forward_equation = (r/4)*[ 1  1  1  1;
                          -1  1  1 -1;
                           1/L 1/-L  1/L 1/-L ];

% Actual Velocity in space calculation
velocity_actual = Forward_equation*wi;

Vx_actual = velocity_actual(1); % Actual velocity in x axis
Vy_actual = velocity_actual(2); % Actual velocity in y axis 
w_actual = velocity_actual(3);  % Actual velocity in rotation

fprintf('\nActual velocity m/s is:\n');
fprintf('Vx = %.3f m/s, Vy = %.3f m/s, w = %.3f rad/s\n', Vx_actual, Vy_actual, w_actual);



%% 

% Trajectory planning (square, circle)

% Square
square_x = [0 1 1 0 0];
square_y = [0 0 1 1 0];

subplot(1, 2, 1);
plot(square_x, square_y, 'b', 'LineWidth', 1.5);
axis equal;
grid on;
title('1m x 1m Square Trajectory');
xlabel('X (m)');
ylabel('Y (m)');

figure;
for i = 2:length(square_x)
    % Linear interpolation between corner points
    x_segment = linspace(square_x(i-1), square_x(i), 50);
    y_segment = linspace(square_y(i-1), square_y(i), 50);
    
    for j = 1:length(x_segment)
        plot(square_x, square_y, 'b--'); hold on;
        plot(x_segment(1:j), y_segment(1:j), 'b', 'LineWidth', 1.5);
        plot(x_segment(j), y_segment(j), 'bo', 'MarkerFaceColor', 'b');
        axis([ -0.2 1.2 -0.2 1.2 ]);
        axis equal;
        grid on;
        title('1m x 1m Square Trajectory');
        xlabel('X (m)');
        ylabel('Y (m)');
        pause(0.03);
        hold off;
    end
end
%% 

% Circle
r = 1; 
theta = linspace(0, 2*pi, 1000);

circle_x = r * cos(theta);
circle_y = r * sin(theta);

subplot(1, 2, 2);
plot(circle_x, circle_y, 'r', 'LineWidth', 1.5);
axis equal;
grid on;
title('1m Diameter Circular Trajectory');
xlabel('X (m)');
ylabel('Y (m)');

figure;
for i = 1:length(circle_x)
    plot(circle_x(1:i), circle_y(1:i), 'b');
    hold on;
    plot(circle_x(i), circle_y(i), 'ro', 'MarkerFaceColor', 'r');
    axis([-1 1 -1 1]);
    axis equal;
    grid on;
    title('Circular Trajectory');
    xlabel('X (m)');
    ylabel('Y (m)');
    pause(0.05);
end

%% 

% spiral

r = 1;              % starting radius
n = 2;              % Number of turns desired
position = 2*pi*n;  % End position
k = r/(2*pi*n);     % decreseing ratio
theta = linspace(0, position, 1000);

r_spiral = r - k * theta;
r_spiral(r_spiral < 0) = 0;

x_spiral = r_spiral .* cos(theta);
y_spiral = r_spiral .* sin(theta);

subplot(1, 2, 2);
plot(x_spiral, y_spiral, 'r', 'LineWidth', 1.5);
axis equal;
grid on;
title('1m Spiral Trajectory');
xlabel('X (m)');
ylabel('Y (m)');

figure;
for i = 1:length(x_spiral)
   
    plot(x_spiral(1:i), y_spiral(1:i), 'r', 'LineWidth', 3);
    hold on;
    
    plot(x_spiral(i), y_spiral(i), 'r', 'MarkerFaceColor', 'r');
    
    axis([-r r -r r]);    
    axis equal;
    grid on;
    title('1m Spiral Trajectory');
    xlabel('X (m)');
    ylabel('Y (m)');
    
    pause(0.05);  
    hold off;
end

%% 

% Pre-determined trajectory linear

% Block location 
Obstacle = [ 4  9   2.5 1 
             1  2   2   5
             5  4   1   3
             9  2   1   2
             12 2   1   2
             9  5.5 1   2
             12 5.5 1   2];  % location in space: [x axis, y axis, width, height]

% Path co-ordinate determine 
x_path = [ 15 8 4  4  2  0 ];
y_path = [  1 1 3 7.5 10 10 ];

% plot Working space
figure; 
x_space = [0  0 15 15 0];
y_space = [0 11 11  0 0];
plot(x_space, y_space,'b--','LineWidth',2); % Plot board
grid on;
hold on;

% Plot block
for i = 1:size(Obstacle,1)
    rectangle('Position',[Obstacle(i,1) Obstacle(i,2) Obstacle(i,3) Obstacle(i,4) ], 'FaceColor',[0.9 0.8 0.2]);
end

% Plot Path
plot(x_path,y_path,'r','LineWidth',3);
plot(x_path,y_path,'ro','LineWidth',3);

title('predetermined Trajectory with Obstacles in 10 m x 15 m Workspace');
xlabel('X Position (m)');
ylabel('Y Position (m)');
grid on;
axis equal;

% Moving animation
for i = 1:length(x_path)
    plot(x_path(1:i), y_path(1:i), 'b', 'LineWidth', 3);   % path
    hold on;
    plot(x_path(i), y_path(i), 'ro', 'MarkerFaceColor', 'r'); % moving point
    axis([0 15 0 15]);
    axis equal;
    grid on;
    xlabel('X (m)');
    ylabel('Y (m)');
    title('predetermined Trajectory Animation');
    pause(0.5);
end


