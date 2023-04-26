function voCallbackFn(~,imgMsg)
    %function [loc, orient, matched_points, inliers] = voCallbackFn(images, start_index, end_index, plot_on)
    % relies on VO_detect_points.m and VO_extract_features.m as used in
    % Experiment 3 for Visual Odometry
    fprintf('Received message\n');
    img = rgb2gray(readImage(imgMsg));
    global rosmsg_handles
    persistent prevImg
    plot_on = 0;
    dt = 1/5; %images are sent at a rate of 5Hz
    focalLength    = [591.1707 592.5926];        % specified in units of pixels
    principalPoint = [316.807 228.4456];        % in pixels [x, y]
    imageSize      = [480 640]; % in pixels [mrows, ncols]
    intrinsics = cameraIntrinsics(focalLength, principalPoint, imageSize);
    
    if isempty(prevImg)
        %intstantiate and publish starting position and orientation
        rosmsg_handles.posePubmsg.Position.X = 0.0;
        rosmsg_handles.posePubmsg.Position.Y = 0.0;
        rosmsg_handles.posePubmsg.Position.Z = 0.0;
        rosmsg_handles.posePubmsg.Orientation.X = 0.0;
        rosmsg_handles.posePubmsg.Orientation.Y = 0.0;
        rosmsg_handles.posePubmsg.Orientation.Z = 0.0;
        rosmsg_handles.posePubmsg.Orientation.W = 1.0;
        send(rosmsg_handles.posePub, rosmsg_handles.posePubmsg)
        rosmsg_handles.markerPubmsg.Header.FrameId = "map";
        rosmsg_handles.markerPubmsg.Header.Stamp = rostime('now');
        rosmsg_handles.markerPubmsg.Type = 0; % Arrow
        rosmsg_handles.markerPubmsg.Action = 0; % Add
        rosmsg_handles.markerPubmsg.Scale.X = 1.0;
        rosmsg_handles.markerPubmsg.Scale.Y = 0.1;
        rosmsg_handles.markerPubmsg.Scale.Z = 0.1;
        rosmsg_handles.markerPubmsg.Color.R = 0.0;
        rosmsg_handles.markerPubmsg.Color.G = 0.0;
        rosmsg_handles.markerPubmsg.Color.B = 1.0;
        rosmsg_handles.markerPubmsg.Color.A = 1.0;
        rosmsg_handles.markerPubmsg.Pose.Position.X = 0.0;
        rosmsg_handles.markerPubmsg.Pose.Position.Y = 0.0;
        rosmsg_handles.markerPubmsg.Pose.Position.Z = 0.0;
        rosmsg_handles.markerPubmsg.Pose.Orientation.X = 0.0;
        rosmsg_handles.markerPubmsg.Pose.Orientation.Y = 0.0;
        rosmsg_handles.markerPubmsg.Pose.Orientation.Z = 0.0;
        rosmsg_handles.markerPubmsg.Pose.Orientation.W = 1.0;
        send(rosmsg_handles.markerPub, rosmsg_handles.markerPubmsg);
        prevImg = img;
    else
        
        % Detect previous image points and features
        prevPoints = VO_detect_points(prevImg);
        prevFeatures = VO_extract_features(prevImg, prevPoints);
        if plot_on
            subplot(2,2,1)
            imshow(prevImg)
            title('Detected points in first view')
            hold on
        
            % plot detected points in first view
            plot(prevPoints.Location(:,1),prevPoints.Location(:,2),'ys'); %detected points
            drawnow
        end

        % Detect current image points and features
        currPoints = VO_detect_points(img);
        currFeatures = VO_extract_features(img, currPoints);
        if plot_on
            subplot(2,2,2)
            imshow(img)
            title('Detected points in second view')
            hold on
        
            % plot detected points in first view
            plot(currPoints.Location(:,1),currPoints.Location(:,2),'ys'); %detected points
            drawnow
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
            imshow(img)
            plot(currPoints.Location(:,1),currPoints.Location(:,2),'m<'); %detected points
            % plot(currPoints.Location(indexPairs(:,2),1),currPoints.Location(indexPairs(:,2),2),'w*'); %matched points
        
        
            
            showMatchedFeatures(img,img,matchedPoints1,matchedPoints2);
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
            showMatchedFeatures(img,img,TrackedPoints1,TrackedPoints2);
            title('Tracked points after motion estimation and outlier rejection')
        end
        
        % return pose and match capability
%         matched_points = numel(indexPairs);
%         inliers = numel(inlierIdx);
        quat = rotm2quat(orient);
        rosmsg_handles.posePubmsg.Position.X = loc(1);
        rosmsg_handles.posePubmsg.Position.Y = loc(2);
        rosmsg_handles.posePubmsg.Position.Z = loc(3);
        rosmsg_handles.posePubmsg.Orientation.X = quat(1);
        rosmsg_handles.posePubmsg.Orientation.Y = quat(2);
        rosmsg_handles.posePubmsg.Orientation.Z = quat(3);
        rosmsg_handles.posePubmsg.Orientation.W = quat(4);
        send(rosmsg_handles.posePub, rosmsg_handles.posePubmsg);
        rosmsg_handles.markerPubmsg.Header.FrameId = "map";
        rosmsg_handles.markerPubmsg.Header.Stamp = rostime('now');
        rosmsg_handles.markerPubmsg.Type = 0; % Arrow
        rosmsg_handles.markerPubmsg.Action = 0; % Add
        rosmsg_handles.markerPubmsg.Scale.X = 1.0;
        rosmsg_handles.markerPubmsg.Scale.Y = 0.1;
        rosmsg_handles.markerPubmsg.Scale.Z = 0.1;
        rosmsg_handles.markerPubmsg.Color.R = 0.0;
        rosmsg_handles.markerPubmsg.Color.G = 0.0;
        rosmsg_handles.markerPubmsg.Color.B = 1.0;
        rosmsg_handles.markerPubmsg.Color.A = 1.0;
        rosmsg_handles.markerPubmsg.Pose.Position.X = loc(1);
        rosmsg_handles.markerPubmsg.Pose.Position.Y = loc(2);
        rosmsg_handles.markerPubmsg.Pose.Position.Z = loc(3);
        rosmsg_handles.markerPubmsg.Pose.Orientation.X = quat(1);
        rosmsg_handles.markerPubmsg.Pose.Orientation.Y = quat(2);
        rosmsg_handles.markerPubmsg.Pose.Orientation.Z = quat(3);
        rosmsg_handles.markerPubmsg.Pose.Orientation.W = quat(4);
        send(rosmsg_handles.markerPub, rosmsg_handles.markerPubmsg);
        prevImg = img;
    
    end
end