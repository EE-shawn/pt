function [VP, lineLabel] = vpDetectionFromLines(lines)

%% Simple vanishing point detection using RANSAC
% === Input === 
% lines: [NumLines x 5]
%   - each row is a detected line segment (x1, y1, x2, y2, length)
% 
% === Output ===
% VP: [2 x 3]
%   - each column corresponds to a vanishing point in the order of X, Y, Z
% lineLabel: [NumLine x 3] logical type
%   - each column is a logical vector indicating which line segments
%     correspond to the vanishing point.

%%
% % Parameters
numVP = 3;
numLines = size(lines,1);
numLinePairSamples = 500;

% Get hypothesis of VPs
M = min(numLinePairSamples, numLines);
vpEst = generateVPHypothesis(lines, M);

% Get preference matrix (check inliers for each VP)
thresInlier = 2;
PrefMat = computePrefMatVP2Lines(vpEst, lines, thresInlier);

% JLinkage clustering
lineLabel = clusterLineSeg(PrefMat);

VP = zeros(2, numVP);
for i = 1: numVP
    PrefMatCurVP = PrefMat(lineLabel(:,i),:);
    score = sum(PrefMatCurVP);
    [~, vpInd] = max(score);
    VP(:,i) = vpEst(vpInd,:)';
end

[VP, lineLabel] = sortVP(VP, lineLabel);

end

function lineLabel = clusterLineSeg(PrefMat)

numVP = 3;

T = JLinkageCluster(PrefMat);
numCluster = length(unique(T));
clusterCount = hist(T, 1:numCluster);
[~, I] = sort(clusterCount, 'descend');

lineLabel = false(size(PrefMat,1), numVP);
for i = 1: numVP
    lineLabel(:,i) = T == I(i);
end

end

function [T, Z, Y] = JLinkageCluster(PrefMat)

Y = pDistJaccard(PrefMat');
Z = linkageIntersect(Y, PrefMat);
T = cluster(Z,'cutoff',1-(1/(size(PrefMat,1)))+eps,'criterion','distance');

end


function [VP_out, lineLabel_out] = sortVP(VP_in, lineLabel_in)

numVP = 3;

% Sort VP to right, left, and top
VP_out = zeros(2, numVP);
lineLabel_out = zeros(size(lineLabel_in));
vpInd = [1,2,3];

[~, zenithInd] = max(abs(VP_in(2,:)));
vpInd(zenithInd) = [];

[~, rightInd] = max(VP_in(1,vpInd));
[~, leftInd]  = min(VP_in(1,vpInd));

permInd = [vpInd(rightInd), vpInd(leftInd), zenithInd];

for i = 1:numVP
    VP_out(:,i) = VP_in(:,permInd(i));
    lineLabel_out(:,i) = lineLabel_in(:,permInd(i));
end

lineLabel_out = logical(lineLabel_out);

end


function PrefMat = computePrefMatVP2Lines(vpEst, lines, thres)

numVPEst = size(vpEst,1);
numLines = size(lines, 1);

PrefMat = false(numLines, numVPEst);

e1 = lines(:,1:2);
e2 = lines(:,3:4);

eMid = (e1 + e2)/2;

for i = 1: numVPEst
    % lineHat: the line connect vpEst(i,:) and eMid
    lineHatAB = linesXY2AB(cat(2, eMid, repmat(vpEst(i,:), numLines, 1)));
    
    % compute the distance of e1 to the lineHat
    dist = distPt2Line(e1, lineHatAB);
    
    %
    PrefMat(:,i) = dist < thres;
end


end

function d = distPt2Line(pt, line)

lineNorm = sqrt(line(:,1).^2 + line(:,2).^2);

d = abs(sum(pt.*line, 2) + 1);
d = bsxfun(@rdivide, d, lineNorm);

end

function vpEst = generateVPHypothesis(lines, M)

% Randomly select lines
randInd1 = randperm(M);
randInd2 = randperm(M);

% Remove invalid samples
validInd = (randInd1 ~= randInd2);
randInd1 = randInd1(validInd);
randInd2 = randInd2(validInd);

% Update M
M = size(randInd1, 2);

% Sampling lines
lineSet1 = lines(randInd1, :);
lineSet2 = lines(randInd2, :);

% Convert lines to a*x + b*y + 1 = 0 representation
linesAB1 = linesXY2AB(lineSet1);
linesAB2 = linesXY2AB(lineSet2);

% Compute vp from sampled line pairs
vpEst = vpEstFromLines(linesAB1, linesAB2, M);


end

function vpEst = vpEstFromLines(linesAB1, linesAB2, M)

A = zeros(2,2);
b = -ones(2,1);
vpEst = zeros(M, 2);
for i = 1:M
    A(1,:) = linesAB1(i,:);
    A(2,:) = linesAB2(i,:);
    vpEst(i,:) = A\b;
end

end

function linesAB = linesXY2AB(lines)
% Convert each row (x1, y1, x2, y2) to line representation a*x + b*y + 1 = 0

numLines = size(lines, 1);

linesAB  = zeros(numLines, 2);

A = zeros(2,2);
b = [-1;-1];
for i = 1: numLines
    A(1,:) = lines(i,1:2);
    A(2,:) = lines(i,3:4);
    linesAB(i,:) = A\b;
end
end