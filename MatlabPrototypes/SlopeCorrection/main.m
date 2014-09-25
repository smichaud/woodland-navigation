clc;
clear all;
close all;
clc;
addpath(genpath('.'));
addpath('../Utils/export_fig');

dataDirectory = '../Data/SlopeCorrection/Natural/';
figuresDirectory = './figures/';
rawData = [];
rawDataStruct = struct(...
    'name', '',...
    'rawMotorCurrents', [],...
    'rawRollPitchYaw', []);
data = [];
dataStruct = struct(...
    'motorCurrents', [],...
    'rollPitchYaw', []);

extractData;

showRawData;