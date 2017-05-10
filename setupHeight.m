function [ VP, eH, p3, p4 ] = setupHeight( frameNum, estHeight, p3, p4)
    fname = strcat(['set0v6/set00_V006_', int2str(frameNum), '.png']);
    imgColor = imread(fname);
    img = rgb2gray(imgColor);
    img = im2double(img);
    lines = lsd(img * 255);
    [VP, ~] = vpDetectionFromLines(lines);

    eH = estHeight;
end

