clear;
clf;
clc;

addpath('./Gabor2/');

inputImage = imread('test.jpg');

% Region of interest (from top left to bottom right)
minX = round(size(inputImage,2)/2 - size(inputImage,2)/4);
maxX = round(size(inputImage,2)/2 + size(inputImage,2)/4);
minY = round(size(inputImage,1)/2);
maxY = size(inputImage,1);

coloredRegion = inputImage(minY:maxY,minX:maxX,:);
imageGray = rgb2gray(coloredRegion);

% Gabor filter
xVariance = 2;
yVariance = 4;
frequency = 16;
orientation = pi/3;
[G,gabout] = gaborfilter1(imageGray,xVariance,yVariance,frequency,pi/3); 

% Display
figure('units','normalized','outerposition',[0 0 1 1])
subplot(1,2,1), imshow(coloredRegion)
subplot(1,2,2), imshow(uint8(gabout));