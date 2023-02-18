function tracks = GetTracks( input, meshSize, demand, nFrames)%inputDir, MeshSize, TracksPerFrame, nFrames
%GetTracks Compute tracks by KLT
%   Use KLT to track evenly fistributed track points
%   input: the path to images
%   meshSize: the meshSize of video stitching
    fileList = dir(input);
    fileList = fileList(3:length(fileList));
    if ~exist('nFrames', 'var') || nFrames > length(fileList)
        nFrames = length(fileList); 
    end
    
    tracks = TrackLib(nFrames);    
    tracks.maxLen = nFrames;
    
    
    tracker = vision.PointTracker('NumPyramidLevels', 3, 'MaxBidirectionalError', 1, 'BlockSize', [51 51]);
    %tracker = vision.PointTracker('MaxBidirectionalError', 1);
    fileName = fileList(1).name;
    frame = imread([input fileName]);
    [H, W, ~] = size(frame);    
    tracks.videoWidth = W;    
    tracks.videoHeight = H;
    
    f_laplacian = fspecial('laplacian',0);
    frame = frame - imfilter(frame, f_laplacian);
    
    livePoints = getMorePoints(frame, meshSize, 0, [], demand);%��һ��Ӧ�þ���ѡ��һЩ�������������ʲô�㷨����Щ�㽫������һ֡�ĳ�ʼ���ٵ�
    initialize(tracker, livePoints, frame);%��ʼ�������Թ��ƣ����ǵ�һ֡��
    tracks.addPoints(livePoints, 1);
    fprintf('%5d', 1);
    for frameIndex = 2:nFrames
        fprintf('%5d', frameIndex);
        if mod(frameIndex, 20) == 0
            fprintf('\n') ;
        end        
        fileName = fileList(frameIndex).name;
        frame = imread([input fileName]);
        
        f_laplacian = fspecial('laplacian',0);%�񻯣����������ӿɼ��ǵ�
        frame = frame - imfilter(frame, f_laplacian);   
        
        
        [livePoints, validity] = step(tracker, frame);%���룺��һ֡�ĸ��ٹ켣����ǰ֡�������ǰ֡�ĸ��ٵ�������Ч������ ��Ч�Ķ���û�ж��ģ����ǣ���
        age = true(size(validity));
        age(tracks.len(tracks.live) == tracks.maxLen) = false;%?��һ�������Ǹ���ģ�
%         fprintf('=> %d\t%d\n', size(tracks.live, 2), size(validity, 1));
        if size(tracks.live, 2) ~= size(validity, 1)
            disp('?') ;
        end
        tracks.endPoints(validity & age, frameIndex);
        tracks.updatePoints(livePoints(validity & age, :), frameIndex);
        
        % end too old tracks 
        morePoints = getMorePoints(frame, meshSize, length(tracks.live), livePoints(validity == true, :), demand);
        tracks.addPoints(morePoints, frameIndex);
        livePoints = [livePoints(validity & age, :); morePoints];
        setPoints(tracker, livePoints);
        marked = insertMarker(frame, livePoints, '*');
        imshow(marked);        
    end
    tracks.endPoints(false(length(tracks.live), 1), length(fileList) + 1);
end

function pointsMore = getMorePoints(frame, meshSize, nP, oldpoints, demand)%frame_s, meshSize, 0, [], demand
    demand = demand / (meshSize * meshSize);%meshSize=16,demand = 2
    votes = zeros(meshSize);
    [H, W, ~] = size(frame);
    threshold = 0.5;
    if nP > 0
        votes = getVotes(frame, meshSize, oldpoints);
    end
    points = [];%���յ�һ֡����np=0
    
    NDIM = ndims(frame);
    
    for row = 1:meshSize
        for col = 1:meshSize
            if votes(row, col) < demand * 0.8
                nMore = floor(demand - votes(row, col));
                roi = [1 + (col - 1) * W / meshSize, 1 + (row - 1) * H / meshSize, W / meshSize - 1, H / meshSize - 1];  
                %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                if NDIM > 2
                    pNew = detectMinEigenFeatures(rgb2gray(frame), 'ROI', roi, 'MinQuality', threshold); 
                else
                    pNew = detectMinEigenFeatures(frame, 'ROI', roi, 'MinQuality', threshold); 
                end
                %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                while (size(pNew, 1) < nMore) && threshold > 0.1
                    threshold = threshold - 0.1; 
                    threshold = max(threshold, 0);
                   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                    if NDIM > 2
                        pNew = detectMinEigenFeatures(rgb2gray(frame), 'ROI', roi, 'MinQuality', threshold); 
                    else
                        pNew = detectMinEigenFeatures(frame, 'ROI', roi, 'MinQuality', threshold); 
                    end
                    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                end
                if nMore < size(pNew, 1)
                    pNew = pNew.selectStrongest(nMore);
                end
                points = [points; pNew.Location];
            end
        end
    end
    pointsMore = points;
    
end

function votes = getVotes(frame, meshSize, points)%ͶƱ
    [H, W, ~] = size(frame);    
    qH = H / meshSize;
    qW = W / meshSize;
    index = floor(points(:, 1) / qW) * meshSize + (floor(points(:, 2) / qH)) + 1; 
    voting = histcounts([index; 1; meshSize*meshSize], meshSize*meshSize);  %ͳ��ֱ��ͼ��Ȼ���һ��1�������ټ�һ��1����һ��meshSize*meshSize,�����ټ�ȥһ��
    voting(1) = voting(1) - 1;
    voting(meshSize * meshSize) = voting(meshSize*meshSize) - 1;    
    votes = reshape(voting, [meshSize meshSize]);%����reshape��Ҳ���ǰ���ֱ��ͼ�ķֲ�����meshSize * meshSizeƽ���֡���
end
