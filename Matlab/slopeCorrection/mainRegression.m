% clear all;
close all;
clc;
addpath(genpath('.'));
addpath('../Utils/export_fig');

figuresDirectory = './figures/';
mkdir('figures');
rawDataStruct = struct(...
    'name', '',...
    'motorCurrents', [],...
    'rollPitchYaw', []);

rawData = [];
dataDirectory = '../Data/SlopeCorrection/Natural/';
extractRawData;
% rawData1 = rawData;
%
% rawData = [];
% dataDirectory = '../Data/SlopeCorrection/Road/';
% extractRawData;
% rawData = [rawData1 ; rawData];

% showRawData;
pitchMotorCurrentsSamples = [];
segmentData;
findRelation;

showMSE;
showLinearRelation;
showPredictions;

slopeCorrection = pitchMotorCurrentsRelation;
save('../Data/SlopeCorrection/slopeCorrection.mat', ...
    'slopeCorrection');

close all;


