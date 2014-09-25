clc;
clear all;
close all;
clc;
addpath(genpath('.'));

dataDirectory = '../Data/SlopeCorrection/Natural/';
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

