function points = VO_detect_points(I)

%% From past experimentation
% numPoints = 150; 
% points   = detectSURFFeatures(I, 'MetricThreshold', 500);
% points   = detectBRISKFeatures(I,'MinContrast',0.25);  %0.21
% points   = detectBRISKFeatures(I,'MinContrast',0.05);
% points   = detectSURFFeatures(I, 'MetricThreshold', 500);
% points   = selectUniform(currPoints, numPoints, size(I));

%% 
numPoints = 200; 
points   = detectORBFeatures(I, 'ScaleFactor',2,'NumLevels',1);  %scale factor 2
% points   = detectSURFFeatures(I, 'MetricThreshold', 500, 'NumOctaves',6);
% points = detectKAZEFeatures(I, 'Threshold',0.0001,'Diffusion','sharpedge','NumOctaves',1);


disp(length(points))
points   = selectUniform(points, numPoints, size(I));


