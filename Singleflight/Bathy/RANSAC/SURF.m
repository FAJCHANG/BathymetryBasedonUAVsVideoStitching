function [source_features] = SURF(I1)
    NDIM = ndims(I1);
    if NDIM > 2 
        grayI1 = rgb2gray(I1);
    else
        grayI1 = I1;
    end

    points1 = detectSURFFeatures(grayI1, 'MetricThreshold', 10, 'NumScaleLevels', 6);


    [f1, vpts1] = extractFeatures(grayI1, points1);

    matched_pts1 = vpts1;


    [n,~] = size(matched_pts1);

    source_features = zeros(n,2);

    for i=1:n
        source_features(i,:) = matched_pts1(i).Location;
    end


end