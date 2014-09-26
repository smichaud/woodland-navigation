clc;
clear all;
close all;
clc;
addpath(genpath('.'));
addpath('../Utils/export_fig');

% dataDirectory = '../Data/SlopeCorrection/Natural/';
dataDirectory = '../Data/SlopeCorrection/Road/';
figuresDirectory = './figures/';
rawData = [];
rawDataStruct = struct(...
    'name', '',...
    'motorCurrents', [],...
    'rollPitchYaw', []);

extractRawData;
showRawData;

data = [];
dataStruct = struct(...
    'motorCurrents', [],...
    'rollPitchYaw', []);

segmentData;


