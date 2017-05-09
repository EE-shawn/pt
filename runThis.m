peopleDetector = vision.PeopleDetector;

for i = 1:865
    fname = strcat(['set0v6/set00_V006_', int2str(i), '.png']);
    img = imread(fname);
    [bboxes,scores] = peopleDetector(img);
    if size(scores, 1) > 0
        img = insertObjectAnnotation(img,'rectangle',bboxes,scores);
    end
    imshow(img);
    pause(0.2);
end


% imshow(img);
% [bboxes,scores] = peopleDetector(img);
% img = insertObjectAnnotation(img,'rectangle',bboxes,scores);
% figure, imshow(img)
