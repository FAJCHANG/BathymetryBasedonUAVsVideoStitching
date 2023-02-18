addpath(genpath('./CoreFunctions/'));
addpath(genpath('./neededData/'));
addpath('mine');
addpath('others')
addpath('mesh');
addpath('RANSAC');
addpath('stitch');
addpath('blend');
addpath('tracks');
addpath('tracks/helpers');
addpath('graph');
addpath('peter');
%%
rootPath = './results/';
foldName = ["downSample/" "filt/" "resMat/" "orthImg/" "gaussFilt/"];
if ~isfolder(rootPath)
    mkdir(rootPath);
end
for i = 1:length(foldName)
    if ~isfolder([rootPath char(foldName(i))])
        mkdir(rootPath,char(foldName(i)));
    end
end
ds_image_savePath = [rootPath char(foldName(1))];
filter_image_savePath = [rootPath char(foldName(2))];
mat_savePath = [rootPath char(foldName(3))];
intrinsics_name = 'P4camera_1080.mat'; %camera intrinsics file name
%you can obtain it by matlab app "camera calibrator" using your camera
%photo or video

fs = 2; %sampling frequency, the downsample step you should do it youself

poolobj = gcp('nocreate');
delete(poolobj);
cd vlfeat-0.9.19/toolbox;
feval('vl_setup');
cd ../..;
%% params
data = '../dataset/';
input_A = 'left/';
input_B = 'right/';
% ---------------    
PointsPerFrame = 1200; 
TracksPerFrame = 600; %   200-600，Because this parameter is more suitable for pictures with a resolution of 1080P
TrackWindowSize = 40; % 
% ---------------%
MeshSize = 8;                       
MaxIte = 15;                         
Smoothness = 4;                      
Cropping = 1;                       
Stitchness = 30;                     
SKIP_BACKGROUND_SEGMENTATION = false; 
RANSAC = true;                       
% ---------------
OutputPadding = 1200;
OutputPath = 'res_demo'; 
print_details = 0;
%% Optical flow method for tracking feature points
if ~exist([data 'tracks' int2str(TracksPerFrame) '.mat'], 'file')
    trackA = GetTracks([data input_A], 10, TracksPerFrame); %
    trackB = GetTracks([data input_B], 10, TracksPerFrame);
    [trackA, trackB] = TrimTracks(trackA, trackB);
    save([data 'tracks' int2str(TracksPerFrame) '.mat'], 'trackA', 'trackB');
else
    load([data 'tracks' int2str(TracksPerFrame) '.mat']);
end
disp('Finished Tracking, ');
%% Clustering and segmentation of background feature points
if ~exist([data 'tracks_' int2str(TrackWindowSize) '_' int2str(TracksPerFrame) '.mat'], 'file')
    if SKIP_BACKGROUND_SEGMENTATION
        trackA.addFakeLabel(TrackWindowSize);
        trackB.addFakeLabel(TrackWindowSize);
    else
        trackA.addLabel(TrackWindowSize);
        trackB.addLabel(TrackWindowSize);
    end
    if trackA.nLabel > trackB.nLabel
        trackB.nLabel = trackA.nLabel;
    else
        trackA.nLabel = trackB.nLabel;
    end
    save([data 'tracks_' int2str(TrackWindowSize) '_' int2str(TracksPerFrame) '.mat'], 'trackA', 'trackB');
else
    load([data 'tracks_' int2str(TrackWindowSize) '_' int2str(TracksPerFrame) '.mat']);
end
disp('Finished Clustering, ');
%% SIFT matching feature points, function: correct homography matrix
if ~exist([data 'ControlPoints' int2str(PointsPerFrame) '.mat'], 'file')
    [CP, ppf] = getControlPoints([data input_A], [data input_B], PointsPerFrame, RANSAC);
    save([data 'ControlPoints' int2str(PointsPerFrame) '.mat'], 'CP', 'ppf');
else
    load([data 'ControlPoints' int2str(PointsPerFrame) '.mat']);
end
disp('Finished SIFT, ');
%% Calculate labels of common feature points
if ~exist([data 'graph' int2str(TracksPerFrame) '.mat'], 'file')
    alpha = 0.01;
    beta = 0.01; 
    maxNlabel = max(trackA.nLabel, trackB.nLabel);
    trackA.nLabel = maxNlabel; trackB.nLabel = maxNlabel;
    [path, graph, goodA, goodB] = GetGraph(trackA, trackB, CP, ppf, alpha, beta);
    backListA = refineTrack(trackA, goodA);
    backListB = refineTrack(trackB, goodB);
    save([data 'graph' int2str(TracksPerFrame) '.mat'], 'path', 'graph', 'backListA', 'backListB');    
else
    load([data 'graph' int2str(TracksPerFrame) '.mat']);
end
disp('Finished GetGraph');
%% Calculate video path
if ~exist([data 'Path' int2str(MeshSize) '.mat'], 'file')
    [pathA] = getPath([data input_A], MeshSize, trackA, backListA);
    [pathB] = getPath([data input_B], MeshSize, trackB, backListB);
    save([data 'Path' int2str(MeshSize) '.mat'], 'pathA', 'pathB');
else
    load([data 'Path' int2str(MeshSize) '.mat']);
end
disp('Finished Camera Path Estimation, ');
%% Refining feature points
if ~exist([data 'ControlPoints' int2str(PointsPerFrame) '_refine.mat'], 'file')
    [CP_refine, ppf_refine] = refineCP(CP, ppf, backListA, backListB, trackA, trackB);
    save([data 'ControlPoints' int2str(PointsPerFrame) '_refine.mat'], 'CP_refine', 'ppf_refine');
else
    load([data 'ControlPoints' int2str(PointsPerFrame) '_refine.mat']);
end
disp('Finished Control Point Refinement, ');
%% Render mosaic effect
number = 1;
if ~exist([data 'stitcher_' int2str(number) '.mat'], 'file')
    stitcher = VideoStitch2([data input_A], [data input_B], pathA, pathB, CP, ppf, Smoothness, Cropping, Stitchness);
    stitcher.init();
    SECOND_ROUND = 5; 
    stitcher.optPath(MaxIte, SECOND_ROUND);
    dbstop if error
    stitcher.render([data OutputPath], ds_image_savePath, OutputPadding);
    save([data 'stitcher_' int2str(number) '.mat'], 'stitcher');
else
    stitcher = load([data 'stitcher_' int2str(number) '.mat']);
end
disp('Finished Stitching, ');
%% Print details (for debug)
if print_details
    PrintLabel([data input_A], trackA, [data '/left_label2/']);
    PrintLabel([data input_B], trackB, [data '/right_label2/']);
    PrintBackground([data '/left/'], trackA, backListA, [data '/left_back/']);
    PrintBackground([data '/right/'], trackB, backListB, [data '/right_back/']);
    PrintFeature([data input_A], [data input_B], CP, CP_refine, [data '/features/']);
    stitcher.renderGrid([data '/res_grid/'], OutputPadding);
end
%% Calibrate Extrinsics parameters of UAV camera
disp('----------step1 start-----------');
disp('开始标定');
disp('Start calibration');
step1.savePath = mat_savePath;
step1.world.gcp_llh = [
    % Here we need to arrange 4~6 GCPs in advance and record their accurate longitude and latitude
    22.59567217 114.87605409 0.98;
    22.59574443 114.87597943 1.10;
    22.59572386 114.87588552 0.82;
    22.59581508 114.87570574 0.83;
    22.59592502 114.87559983 1.02;
    22.59596176 114.87544107 0.87
];
step1.world.o_llh = [22.59567217 114.87605409 0.98];%Set the world coordinate origin, you can select the first ground control point
step1.world.uav_llh = [22.596327 114.875994 81.62];%The initial position where the UAV starts recording video

step1.intrinsic = ['./neededData/',intrinsics_name]; % Load camera intrinsics parameters
step1.imagePath = ds_image_savePath; 
uavInfo_world = gcpllh2NED(step1.world.o_llh, step1.world.uav_llh);
step1.extrinsicsInitialGuess = [uavInfo_world' deg2rad(-36.0) deg2rad(0) deg2rad(-146.3)];   
step1.extrinsicsKnownsFlag = [0,0,0,0,0,0]; 
[gcpUV, Expara] = chooseGCP(step1);  %This part is mainly to obtain the pixel position of GCPs and extrinsics of first frame
disp('----------step1 end----------');
%% 第二步，通过绑定视频路径来验证是否正确,但是不知道会不会发散，一般不会
%The second step is to bind the video path to verify whether it is correct, but I don't know whether it will diverge, generally not
%如果发散了可能是环境光出现问题了，这个时候你可能需要别的方法来识别控制点，比如说颜色阈值法、模板匹配等等，但本文并未提供
%If there is divergence, it may be that there is a problem with the ambient light. 
%At this time, you may need other methods to identify control points, such as color threshold method, template matching, etc., 
%but this code does not provide
disp('----------step2 start-----------')
threshold = 0.02;% You can adjust the threshold size to identify the background point // 你可以调整大小来识别背景点
if ~exist([data 'pathAone_' int2str(threshold) '.mat'], 'file')
    trackAA = GetTracks([data input_A], 1, TracksPerFrame);
    trackAA.addLabel(TrackWindowSize);
    [graphA, goodAone] = OneGraph(trackAA, threshold);
    backListAone = refineTrack(trackAA, goodAone);
    save([data 'graphA' int2str(TracksPerFrame) '.mat'], 'graphA', 'goodAone', 'backListAone');
    PrintBackground([data '/left/'], trackAA, backListAone, [data '/background/']);
    [pathAone] = getPath([data input_A], 1, trackAA, backListAone);
    save([data 'pathAone_' int2str(threshold) '.mat'], 'pathAone');
    step2.path = pathAone;
    save([data 'tracks_AA' int2str(TrackWindowSize) '_' int2str(TracksPerFrame) '.mat'], 'trackAA');
else
    load([data 'pathAone' int2str(threshold) '.mat']);
    step2.path = pathAone;
end
% [pathAone] = getPath([data input_A], 1, trackA, backListA);
disp('finish one background identification')
%%
step2.savepath = rootPath;
step2.inputDir = ds_image_savePath;
% step4.inputDir = '../dataset/left/';
step2.intrinsic = ['./neededData/',intrinsics_name];
step2.gcpUV = gcpUV;
step2.Expara = Expara;

if ~exist([mat_savePath 'info.mat'], 'file')
    info = calcExtrinsics(step2);
    save([mat_savePath 'info.mat'], 'info');
else
    load([mat_savePath 'info.mat']);
    disp('loading info ... ')
end
disp('----------step2 end----------');

%% debug:显示选取的ROI区域 Show selected ROI area
dbg = 1;
if dbg == 1
    debug.info = info;
    debug.intrinsics = load(['./neededData/',intrinsics_name]);
    debug.ix = [0, 100];
    debug.iy = [0, 100];
    debug.resolution = 0.5;
    debug.inputdir = ds_image_savePath;
    debug.savepath = mat_savePath;
    debug.index = 100;
    debug.local_angle = -148.5;
    debug.gcp_llh = step1.world.gcp_llh;
    debug.o_llh = step1.world.o_llh;
    chooseROIshow(debug);
end
%% 这一步要将区域提取出来,正射变换  In this step, we need to extract the region, orthomorphic transformation
%此外需要声明的是，需要提前将视频拼接的图片进行融合，替换downsample文件，可以使用加权融合的方式，但本文并未提供
%In addition, it should be stated that the image of video mosaic needs to be fused in advance to replace the downsample file. 
%The weighted fusion method can be used, but this code does not provide
disp('----------step3 start----------')

step3.savepath = mat_savePath;
step3.saverotpath = [rootPath char(foldName(4))];
step3.inputdir = ds_image_savePath;
step3.filt_path = filter_image_savePath;

step3.local_angle = -148.5;
step3.local_origin = [0,0];
step3.pixel_resolution = 0.5;
step3.intrinsic = ['./neededData/',intrinsics_name];
step3.gcps = step1.world.gcp_llh;

pixelImg = getPixelImage(step3, info);
disp('----------step3 finished!----------');
%% 旋转并导出图片 Rotate and export pictures
disp('----------step4 start----------');

step4.pixelpath = [mat_savePath 'pixelImg.mat'];
step4.savePath = [rootPath char(foldName(4))];
if ~isfolder(step4.savePath)
     mkdir(rootPath,char(foldName(4)));
end
rotImg(step4);
disp('----------step4 finish--------------- ');
disp('----------ALL STEP FINISH!!--------------- ');