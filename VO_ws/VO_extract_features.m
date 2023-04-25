function features = VO_extract_features(I, points)

%% From past experimentation
% features = extractFeatures(I, points, 'Upright', true);
% features = extractFeatures(I, points, 'BRISK', true);


%% Sage's experimentation:
features = extractFeatures(I, points, 'Method', 'ORB');  
% features = extractFeatures(I, points, 'Method', 'SURF');
% features = extractFeatures(I, points, 'Method', 'KAZE');


features;