clear all;
close all;
clc;
%% Tracking multiple objects
obj_detect.videoPlayer = vision.VideoPlayer('Position', [740, 400, 800, 800]);  % to show detections 
obj_track.videoPlayer = vision.VideoPlayer('Position', [20, 400, 800, 800]);    % to show tracking
%% Initialize tracks
tracks = struct(...
            'id', {}, ...
            'bbox', {}, ...
            'x', {}, ...                            % For states
            'P',{}, ...                             % For error covariance matrix
            'age', {}, ...          
            'totalVisibleCount', {}, ...
            'consecutiveInvisibleCount', {});
%% Constants and initialization
dt = 1/30;                                          % frame rate
% Constant velocity model
% states = [x x_dot y y_dot]'
A = [1 dt 0 0;
     0 1 0 0;
     0 0 1 dt;
     0 0 0 1];
% measuring the position of the object (centroid in this case)
% z = [x y]'
H = [1 0 0 0;
     0 0 1 0];
% Initialization 
x0 = [0,0,0,0]'; P0 = 100*eye(4);       % initial guess for states a

% Change these values to tune kalman filter 
q = [20,20,20,20];                      % process noise, denotes trust level
r = [15,15];                            % measurement noise

nextId = 1;

peopleDetector = vision.PeopleDetector; 
%% Main loop
isDetection = false;                    % correction step is performed when a measurement is available

for j = 1:885
    fname = strcat(['set0v6/set00_V006_', int2str(j), '.png']);
    img = imread(fname);
    frame = img;
    [bboxes,scores] = step(peopleDetector, img);
    if size(scores,1)>0 % valid detection
        isDetection = true;
        centroids = [bboxes(:,1)+bboxes(:,3)/2 bboxes(:,2)+bboxes(:,4)/2];  % centroids of detections
        img = insertObjectAnnotation(img,'rectangle',bboxes,scores);
        obj_detect.videoPlayer.step(img)                                    % to show detections
    end
    %% Predict new locations
    for i = 1:length(tracks)
        bbox = tracks(i).bbox;
        % Predict the current location of the track
        
        [x_pred,P_pred] = predict_test(A,H,tracks(i).x,tracks(i).P,q);
        predCentroid = [x_pred(1) x_pred(3)];           % predicted centroid
        x0 = x_pred;
        P0 = P_pred;
        tracks(i).x = x0;
        tracks(i).P = P0;
        predCentroid = (predCentroid) - bbox(3:4) / 2;
        tracks(i).bbox = [predCentroid, bbox(3:4)];
    end
    %% assigned and unassigned tracks, check the function
    if isDetection
    nTracks = length(tracks);
    nDetections = size(centroids, 1);
    
    % Compute the cost of assigning each detection to each track.
    cost = zeros(nTracks, nDetections);
    for i = 1:nTracks
        pos = tracks(i).x;
        cost(i, :) = distance([pos(1) pos(3)], centroids);
    end
    
    % Solve the assignment problem.
    costOfNonAssignment = 50;
    [assignments, unassignedTracks, unassignedDetections] = ...
        assignDetectionsToTracks(cost, costOfNonAssignment);
    %% update assigned tracks
    numAssignedTracks = size(assignments,1);
    for i = 1:numAssignedTracks
        trackIdx = assignments(i, 1);
        detectionIdx = assignments(i, 2);
        centroid = centroids(detectionIdx, :);
        bbox = bboxes(detectionIdx, :);
        
        % Correct the estimate of the object's location
        % using the new detection.
        z = centroid';
        
        [x_corr,P_corr] = correct_test(H,tracks(trackIdx).x,tracks(trackIdx).P,z,r);
        
        % Replace predicted bounding box with detected
        % bounding box.
        tracks(trackIdx).bbox = bbox;
        tracks(trackIdx).x = x_corr;
        tracks(trackIdx).P = P_corr;
        % Update track's age.
        tracks(trackIdx).age = tracks(trackIdx).age + 1;
        
        % Update visibility.
        tracks(trackIdx).totalVisibleCount = ...
            tracks(trackIdx).totalVisibleCount + 1;
        tracks(trackIdx).consecutiveInvisibleCount = 0;
    end
    %% update unassigned tracks
    for i = 1:length(unassignedTracks)
        ind = unassignedTracks(i);
        tracks(ind).age = tracks(ind).age + 1;
        tracks(ind).consecutiveInvisibleCount = ...
            tracks(ind).consecutiveInvisibleCount + 1;
    end
     %% delete lost tracks
    invisibleForTooLong = 20;
    ageThreshold = 5;
    
    % Compute the fraction of the track's age for which it was visible.
    ages = [tracks(:).age];
    totalVisibleCounts = [tracks(:).totalVisibleCount];
    visibility = totalVisibleCounts ./ ages;
    
    % Find the indices of 'lost' tracks.
    lostInds = (ages < ageThreshold & visibility < 0.6) | ...
        [tracks(:).consecutiveInvisibleCount] >= invisibleForTooLong;
    
    % Delete lost tracks.
    tracks = tracks(~lostInds);
    %% based on detection create new tracks
    centroids = centroids(unassignedDetections, :);
    bboxes = bboxes(unassignedDetections, :);
    
    for i = 1:size(centroids, 1)
        
        centroid = centroids(i,:);
        bbox = bboxes(i, :);
        
        % Create a Kalman filter object.        
        % Create a new track.
        x0 = [centroid(1) 0 centroid(2) 0]';
        newTrack = struct(...
            'id', nextId, ...
            'bbox', bbox, ...
            'x',x0, ...
            'P',P0, ...
            'age', 1, ...
            'totalVisibleCount', 1, ...
            'consecutiveInvisibleCount', 0);
        
        % Add it to the array of tracks.
        tracks(end + 1) = newTrack;
        
        % Increment the next id.
        nextId = nextId + 1;
    end
 %% Display results
    minVisibleCount = 8;
    if ~isempty(tracks)
        
        % Noisy detections tend to result in short-lived tracks.
        % Only display tracks that have been visible for more than
        % a minimum number of frames.
        reliableTrackInds = ...
            [tracks(:).totalVisibleCount] > minVisibleCount;
        reliableTracks = tracks(reliableTrackInds);
        
        % Display the objects. If an object has not been detected
        % in this frame, display its predicted bounding box.
        if ~isempty(reliableTracks)
            % Get bounding boxes.
            bboxes = cat(1, reliableTracks.bbox);
            
            % Get ids.
            ids = int32([reliableTracks(:).id]);
            
            % Create labels for objects indicating the ones for
            % which we display the predicted rather than the actual
            % location.
            labels = cellstr(int2str(ids'));
            predictedTrackInds = ...
                [reliableTracks(:).consecutiveInvisibleCount] > 0;
            isPredicted = cell(size(labels));
            isPredicted(predictedTrackInds) = {' Predicted'};
            labels = strcat(labels, isPredicted);
            
            ind = strcmp(isPredicted,' Predicted');      % find the indices for prediction
            % Display the object, predicted objects are shown in red
            if (sum(ind))
                color = cell(1,length(labels));
                color(:) = {'yellow'};
                color(ind) = {'red'};
                frame = insertObjectAnnotation(frame, 'rectangle', ...      % if prediction
                        bboxes, labels,'Color',color);
            else
                frame = insertObjectAnnotation(frame, 'rectangle', ...      % if no prediction
                    bboxes, labels);
            end
        end
    end
    obj_track.videoPlayer.step(frame);
    end
    isDetection = false;
end
