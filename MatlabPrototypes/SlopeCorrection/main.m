clc;
clear all;
close all;
clc;
addpath(genpath('.'));

dataDirectory = '../Data/';
dataStruct = struct(...
    'name', '',...
    'rawCurrents', [],...
    'rawRollPitchYaw', []);

extractData;

