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
% for i = 160: 1700
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
%     
%     disp(i);
% end

fname = strcat(['set0v6/set00_V006_', int2str(250), '.png']);
imgColor = imread(fname);
  
figure(1);
imshow(imgColor);
[x,y] = getpts;

[VP, eH, p3, p4] = setupHeight(162, 1.7, [430, 80], [430, 350]);

p1 = [x(1), y(1)];
p2 = [x(2), y(2)];

h = estimateHeight(VP, eH, p1, p2, p3, p4);
disp('Height:')
disp(h)