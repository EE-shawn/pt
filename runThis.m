close all;

peopleDetector = vision.PeopleDetector;

figure(1);
% Run people detector on all frames of the video
for i = 1050:1865
    fname = strcat(['set0v6/set00_V006_', int2str(i), '.png']);
    img = imread(fname);
    [bboxes,scores] = peopleDetector(img);
    if size(scores, 1) > 0
        img = insertObjectAnnotation(img,'rectangle',bboxes,scores);
    end
    imshow(img);
    pause(0.2);
end
