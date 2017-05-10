%% Evaluation script for vanishing point detection and camera calibration

% 
% Jia-Bin Huang
% University of Illinois, Urbana-Champaign
% www.jiabinhuang.com

%%
clear
addpath('lsd-1.5');
addpath('JLinkage');

% A subset of YorkUrban dataset
datasetPath = 'set0v6';

% Image list
imgDir = dir(fullfile(datasetPath, '*.png'));
numImg = length(imgDir);

% Process imagess
% for i = 1050: 1700
%     disp(i);
%     fname = strcat(['set0v6/set00_V006_', int2str(i), '.png']);
%    
%     % Load image
%     imgColor = imread(fname);
%     img = rgb2gray(imgColor);
%     img = im2double(img);
%     
%     % Line segment detection
%     lines = lsd(img * 255);
%     
%     % Vanishing point detection from line segments
%     [VP, lineLabel] = vpDetectionFromLines(lines);
%        
%     % Camera calibration
%     [f, u0, v0, R] = camCalibFromVP(VP);
%     
%     % === Visualization ===
%     visLineSegForVP(imgColor, lines, lineLabel, VP, img);
% end

fname = strcat(['set0v6/set00_V006_', int2str(1054), '.png']);
imgColor = imread(fname);
img = rgb2gray(imgColor);
img = im2double(img);
lines = lsd(img * 255);
[VP, lineLabel] = vpDetectionFromLines(lines);

figure(1);
imshow(imgColor);
[x,y] = getpts;

p1 = [x(1), y(1)];
p2 = [x(2), y(2)];

disp(p2);

h = estimateHeight(VP, p1, p2);
disp('Height:')
disp(h)