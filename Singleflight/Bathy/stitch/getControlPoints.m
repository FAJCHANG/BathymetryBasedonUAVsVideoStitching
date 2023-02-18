function [CP, ppf] = getControlPoints( input_A, maxppf, ransac)
    disp('Detecting SIFT features...');
    fileListA = dir(input_A);
    fileListA = fileListA(3:length(fileListA));
    nFrames = length(fileListA);
    CP = zeros(nFrames, maxppf, 2);
    ppf = zeros(nFrames, 1);
    trackerA = vision.PointTracker('MaxBidirectionalError', 1);
    
    for frameIndex = 1:nFrames
        fprintf('%5d', frameIndex);
        if mod(frameIndex, 20) == 0
            fprintf('\n') ;
        end

        fileNameA = fileListA(frameIndex).name;
        IA = imread([input_A fileNameA]);
        [H, W, ~] = size(IA);

        
        if frameIndex > 1
            setPoints(trackerA, trackA);
            [trackAcont, validityA] = step(trackerA, IA);
            trackAcont = trackAcont(validityA, :);
        end
        if ransac
            [trackAsurf] = SURF(IA);
        else
            [trackAsurf] = SURF2(IA);
        end

        trackA = trackAsurf;
        
        valid = trackA(:, 1) > 0 & trackA(:, 1) < W & trackA(:, 2) > 0 & trackA(:, 2) < H;
        
        trackA = trackA(valid, :);

        valid = filtermask(IA, trackA);
        trackA = trackA(valid, :);

        if frameIndex == 1
            initialize(trackerA, trackA, IA);
        else
            if length(trackA) + length(trackAcont) > maxppf
                ordering = randperm(length(trackAcont));%这个MATLAB函数返回一个行向量，其中包含从1到n的整数的随机排列。
                trackAcont = trackAcont(ordering(1:maxppf - length(trackA)), :);
            end
            trackA = [trackA ; trackAcont];
        end
        
        IA = insertMarker(IA, trackA, 'o', 'color', 'red');%这个MATLAB函数返回带有插入的加(+)标记的真彩色图像。
        figure(1);
        imshow(IA);


        if length(trackA) > maxppf
            ppf(frameIndex) = maxppf;
            CP(frameIndex, :, 1:2) = trackA(1:maxppf, :);
        else
            ppf(frameIndex) = length(trackA);
            CP(frameIndex, 1:ppf(frameIndex), 1:2) = trackA;
        end
%         CP(frameIndex, :, :) = [featuresA featuresB; zeros(maxppf - ppf(frameIndex), 4)];

    end
end

function valid = filtermask(frame, points_)
    [H, W, ~] = size(frame);
    valid = points_(:, 1) > 0 & points_(:, 1) < W & points_(:, 2) > 0 & points_(:, 2) < H;
    points_ = points_(valid, :);
    mask = frame(:, :, 1) < 20;% & frame(:, :, 2) < 20 & frame(:, :, 3) < 20;%%%
    mask = imgaussfilt(double(mask), 50);
    videoH = size(frame, 1);
    mask(mask > 0.2) = 1;
    mask(mask ~= 1) = 0;
    valid = mask(round(points_(:, 1) - 1) * videoH + round(points_(:, 2))) == 0;
end


