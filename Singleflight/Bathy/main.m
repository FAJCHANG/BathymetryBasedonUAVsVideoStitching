%%图像预处理
addpath(genpath('./CoreFunctions/'));
addpath(genpath('./neededData/'));
addpath('mine');
rootPath = './results/';
foldName = ["downSample/" "filt/" "resMat/" "orthImg/" "gcpsPos/" "input/" "tmp/"];
if ~isfolder(rootPath)
    mkdir(rootPath);
end
for i = 1:length(foldName)
    if ~isfolder([rootPath char(foldName(i))])
        mkdir(rootPath,char(foldName(i)));
    end
end
ds_image_savePath = [rootPath char(foldName(1))];%保存下采样图片
filter_image_savePath = [rootPath char(foldName(2))];%保存高斯滤波图
mat_savePath = [rootPath char(foldName(3))];%保存mat文件
gaussian_kernel = 50;
intrinsics_name = 'P4camera_1080.mat';
fs = 2;
%% 第一步下采样 Step 1 Sampling
disp('----------step1 DwonSample start----------');
ignore = 0;
if ignore
    videopath = "../Video/DJI_0003.MOV"; % 更改需要下采样的视频文件路径
    command = 'python ./downSample.py -v "' + videopath + '"' + ' -hz ' + num2str(fs) + '-kernel' + num2str(gaussian_kernel);
    status = system(command);
    disp('downSample Finished');
end
disp('----------step1 end----------')

%% 第二步绑定视频路径 参数项  Step 2 Bind video path parameter items
disp('----------step2 Bundle Video Path Start----------');
addpath('RANSAC');
addpath('tracks');
addpath('tracks/helpers');
addpath('mesh');
addpath('peter');
addpath('graph');
addpath('stitch');
savepath = mat_savePath;

nFrames = length(dir(strcat(ds_image_savePath, '/*.jpg')));
SKIP_BACKGROUND_SEGMENTATION = false;
%-----TRACK----
TracksPerFrame = 250;% 可以调整的参数
TrackWindowSize = 40;
MeshSize = 1; %可以调整的参数 1~10 ，最好能被图片的高宽整除

%%
if ~exist([savepath 'tracks.mat'], 'file')
    trackA = GetTracks(ds_image_savePath, MeshSize, TracksPerFrame);
    save([savepath 'tracks' '.mat'], 'trackA');
else
    load([savepath 'tracks' '.mat']);
end
disp('Finished Tracking, ');
%%
if ~exist([savepath 'tracks_' int2str(TrackWindowSize) '_' int2str(TracksPerFrame) '.mat'], 'file')
    if SKIP_BACKGROUND_SEGMENTATION
        trackA.addFakeLabel(TrackWindowSize);
    else
        trackA.addLabel(TrackWindowSize);
    end
    save([savepath 'tracks_' int2str(TrackWindowSize) '_' int2str(TracksPerFrame) '.mat'], 'trackA');
else
    load([savepath 'tracks_' int2str(TrackWindowSize) '_' int2str(TracksPerFrame) '.mat']);
end
disp('Finished Clustering, ');
%% background identification
% 背景识别，如果识别效果很差，可以使用颜色阈值法或者模板匹配，但本文并未提供
% if the recognition effect is poor, color threshold method or template matching can be used, but this code does not provide
threshold = 0.02; %根据实际调整的参数
if ~exist([savepath 'pathAone' int2str(1) '.mat'], 'file')
    [graphA, goodAone] = OneGraph(trackA, threshold);
    backListAone = refineTrack(trackA, goodAone);
    save([savepath 'graphA' int2str(TracksPerFrame) '.mat'], 'graphA', 'goodAone', 'backListAone');
    PrintBackground(ds_image_savePath, trackA, backListAone, [rootPath '/background/']);
    [pathAone] = getPath(ds_image_savePath, 1, trackA, backListAone);
    save([savepath 'pathAone' int2str(1) '.mat'], 'pathAone');
else
    load([savepath 'pathAone' int2str(1) '.mat']);
end
disp('finish one background identification')
%% 第三步记录控制点，计算外参
% Step 3 Record control points and calculate extrinsics parameters
disp('----------step3 start-----------');
disp('开始标定');
step3.savePath = mat_savePath;

step3.world.gcp_llh = [
    22.59597440 114.87604752 2.08;
    22.59599842 114.87592532 2.17;
    22.59608425 114.87593422 1.96;
    22.59606851 114.87579587 2.14;
    22.59619867 114.87567679 1.98;
    22.59619766 114.87553950 2.16
];
step3.world.o_llh = [22.59597440 114.87604752 2.08];% World origin
step3.world.uav_llh = [22.59648368 114.87615432 77.30];% UAV initial position
step3.intrinsic = ['./neededData/',intrinsics_name]; 
step3.imagePath = ds_image_savePath;

uavInfo_world = gcpllh2NED(step3.world.o_llh,step3.world.uav_llh);

step3.extrinsicsInitialGuess = [uavInfo_world' deg2rad(-36.0) deg2rad(0) deg2rad(-147.6)]; %It needs to be adjusted according to the camera position and Euler angles(pitch roll yaw)   
step3.extrinsicsKnownsFlag = [0,0,0,0,0,0]; 

[gcpUV, Expara] = chooseGCP(step3);
disp('----------step3 end----------');
%% 第四步，计算每一帧外参
% Calculate each frame extrinsics parameters
step4.savepath = rootPath;
step4.inputDir = ds_image_savePath;
step4.intrinsic = ['./neededData/',intrinsics_name];
step4.gcpUV = gcpUV;
step4.Expara = Expara;
step4.path = pathAone;

info = calcExtrinsics(step4);
save([savepath 'info.mat'], 'info');
disp('step4 finished');
%% 这一步要将ROI区域提取出来
% This step extracts the ROI region
disp('----------step5 start----------')

step5.savepath = savepath;
step5.saverotpath = [rootPath char(foldName(4))];
step5.inputdir = ds_image_savePath;
step5.filt_path = filter_image_savePath;

step5.local_angle = -148.5;
step5.local_origin = [0,0];
step5.pixel_resolution = 0.5;
step5.intrinsic = ['./neededData/',intrinsics_name];
step5.gcps = step3.world.gcp_llh;
step5.gaussian_kernel = gaussian_kernel;

[pixelImg] = getPixelImage(step5, info);
disp('step5 finish!');

%% 旋转，可选 Rotation, optional
disp('----------step6 start----------');

step6.pixelpath = [savepath 'pixelImgo.mat'];
step6.savePath = [rootPath char(foldName(4))];
if ~isfolder(step6.savePath)
     mkdir(rootPath,char(foldName(4)));
end
rotImg(step6);

disp('----------step6 finish--------------- ');
disp('----------ALL STEP FINISH!!--------------- ');




