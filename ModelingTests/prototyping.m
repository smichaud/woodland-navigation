% Cylinder is represented by 5 parameters
% n: number of points in the cluster
% f_i: distance of the point from the surface (i.e. residual)

% Parameters (5):
% alpha: roll angle
% beta: pitch angle
% x_t: x coordinate of intersection for X-Y plane
% y_t: y coordinate of intersection for X-Y plane
% r: radius of the cylinder 

% Square residual
% s = sum_i=1^n (sqrt(x^2 + y^2)-r)^2

% Constraints
%  r <= 0.75m
% -0.45rad <= alpha, beta <= 0.45rad

clear;
clc;

% Create cylinder
radius = 1.25;
minZ = -3;
maxZ = 3;
nbPoints = 30;
cylinder = createCylinder(radius,minZ,maxZ,nbPoints);

% Add noise
mu = 0;
sigma = 0.05;
noise = normrnd(mu,sigma,[3,nbPoints]);
cylinder = cylinder + noise;

% Rotate cylinder
roll = 45;
pitch = 45;
unit = 'deg';
cylinder = rotateRollPitch(cylinder, roll, pitch, unit);

% Translate cylinder
translation = [5;5;0];
cylinder = cylinder + repmat(translation,1,nbPoints);

% Plot cylinder
scatter3(cylinder(1,:), cylinder(2,:), cylinder(3,:), 'r.');
axis equal;
hold on;


