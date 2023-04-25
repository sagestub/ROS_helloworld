function voCallbackFn(~,imgMsg)
%function [loc, orient, matched_points, inliers] = voCallbackFn(images, start_index, end_index, plot_on)
% relies on VO_detect_points.m and VO_extract_features.m as used in
% Experiment 3 for Visual Odometry
global lastView rosmsg_handles
dt = 0.2;

if lastView.isnull()
    %intstantiate and publish starting position and orientation
    rosmsg_handles.posePubmsg.Position.X = 0.0;
    rosmsg_handles.posePubmsg.Position.Y = 0.0;
    rosmsg_handles.posePubmsg.Position.Z = 0.0;
    rosmsg_handles.posePubmsg.Orientation.X = 0.0;
    rosmsg_handles.posePubmsg.Orientation.Y = 0.0;
    rosmsg_handles.posePubmsg.Orientation.Z = 0.0;
    rosmsg_handles.posePubmsg.Orientation.W = 1.0;
    send(rosmsg_handles.posePub, rosmsg_handles.posePubmsg)
    lastView = rosmsg_handles.posePubmsg;
else

% Select which views to use (indices of images in sequence)
viewIds = [start_index end_index];

%Create a View Set Containing the First View of the Sequence

% Read the first image.
Irgb = readimage(images, viewIds(1));

% Create the camera intrinsics object using camera intrinsics from the 
% New Tsukuba dataset.
focalLength    = [615 615];        % specified in units of pixels
principalPoint = [320 240];        % in pixels [x, y]
imageSize      = size(Irgb,[1,2]); % in pixels [mrows, ncols]
intrinsics = cameraIntrinsics(focalLength, principalPoint, imageSize);

% Convert to gray scale and undistort. In this example, undistortion has no 
% effect, because the images are synthetic, with no lens distortion. However, 
% for real images, undistortion is necessary.
prevI = undistortImage(rgb2gray(Irgb), intrinsics); 


% Detect features points
prevPoints = VO_detect_points(prevI);

% Extract feature vectors
prevFeatures = VO_extract_features(prevI, prevPoints);

% Visualize detected points from image 1
if plot_on
    subplot(2,2,1)
    imshow(prevI)
    title('Detected points in first view')
    hold on

    % plot detected points in first view
    plot(prevPoints.Location(:,1),prevPoints.Location(:,2),'ys'); %detected points
    drawnow
end


%Extract features from the Second View
% Detect and extract features from the second view, and match them to the first 

% Read the second image.
Irgb = readimage(images, viewIds(2));

% Convert to gray scale and undistort.
I = undistortImage(rgb2gray(Irgb), intrinsics);




% Detect feature points from second image
currPoints = VO_detect_points(I);

% Extract feature vectors from the second image
currFeatures = VO_extract_features(I, currPoints);

% plot detected points in second view
if plot_on
    subplot(2,2,2)
    imshow(I)
    title('Detected points in second view')
    hold on
    plot(currPoints.Location(:,1),currPoints.Location(:,2),'ys'); %detected points
end

%Extract pose from second view by matching features between the 
%   previous and current image.

% Estimate the pose of the second view relative to the first view using <matlab:edit('helperEstimateRelativePose.m') 
indexPairs = matchFeatures(prevFeatures, currFeatures, 'Unique', true);
% indexPairs = matchFeatures(prevFeatures, currFeatures, 'Unique', true, 'Method', 'Approximate', 'MatchThreshold', 50);  %some experimenting

matchedPoints1 = prevPoints(indexPairs(:, 1));
matchedPoints2 = currPoints(indexPairs(:, 2));

if plot_on
    subplot(2,2,3)
    title('Matched features from first to second view')
    
    hold on
    imshow(I)
    plot(currPoints.Location(:,1),currPoints.Location(:,2),'m<'); %detected points
    % plot(currPoints.Location(indexPairs(:,2),1),currPoints.Location(indexPairs(:,2),2),'w*'); %matched points


    
    showMatchedFeatures(I,I,matchedPoints1,matchedPoints2);
end
   
% Estimate the pose of the current view relative to the previous view.
[orient, loc, inlierIdx] = helperEstimateRelativePose(...
    matchedPoints1, matchedPoints2, intrinsics);
    
% Exclude epipolar outliers.
indexPairs = indexPairs(inlierIdx, :);
  
if plot_on
    subplot(2,2,4)
    TrackedPoints1 = prevPoints(indexPairs(:, 1));
    TrackedPoints2 = currPoints(indexPairs(:, 2));
    showMatchedFeatures(I,I,TrackedPoints1,TrackedPoints2);
    title('Tracked points after motion estimation and outlier rejection')
end

% return pose and match capability
matched_points = numel(indexPairs);
inliers = numel(inlierIdx);
end
end