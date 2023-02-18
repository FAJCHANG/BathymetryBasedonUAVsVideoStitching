function gt = getGroundTruth(gt_path, mat_savePath, ds_image_savePath, xyz, o_llh, bathy, tide)
    data = load(gt_path.path1);
    objectPoints = gcpllh2NED(o_llh, data);
    objectPoints = objectPoints';
    Rotate_ned2cs = Euler2Rotate(-150,0,0); %之前是148,
    Rotate_ned2cs = Rotate_ned2cs';
    objectPointsInCs = Rotate_ned2cs*objectPoints';
    objectPointsInCs = objectPointsInCs'; %这一步转换为自建坐标系下的坐标


    %%%% 2、和图上的坐标对应起来
    % 将其转换为坐标

    % Load and Display initial Oblique Distorted Image

    roi_path = [mat_savePath 'GRID_roiInfo0.mat'];
%     extrinsicFullyInfo_path = [mat_savePath 'extrinsicFullyInfo.mat'];
    extrinsicFullyInfo_path = [mat_savePath 'info.mat'];
    intrinsics = load('P4camera_1080.mat');
    intrinsics = intrinsics.intrinsics;
    L= dir(ds_image_savePath);
    L=L(3:end); % 前两个为当前目录.和上级目录..
    I=imread(strcat(ds_image_savePath, L(1).name));
    figure;
    hold on;

    imshow(I);
    hold on;
    title('无人船测量轨迹');
%     load(roi_path);
    ex = load(extrinsicFullyInfo_path);
    ex = ex.info;
    ex = ex(1,:).extrinsics;%f添加
%     Extrinsics=localTransformExtrinsics([0,0], -148.5 ,1, ex.extrinsics);
    Extrinsics=localTransformExtrinsics([0,0], -148.5 ,1, ex);
    extrinsics_ff=Extrinsics(1,:);
%     [step7.UVd] = xyz2DistUV(ex.intrinsics, extrinsics_ff, objectPointsInCs); %利用chooseRoi中选定的区域所具有的xyz信息插值后得到z，并用该值计算uv坐标
    [step7.UVd] = xyz2DistUV(intrinsics, extrinsics_ff, objectPointsInCs);
    step7.UVd = reshape(step7.UVd, [], 2);
    plot(step7.UVd(:,1), step7.UVd(:,2), '*');
%     xlim([0 ex.intrinsics(1)]);
%     ylim([0 ex.intrinsics(2)]);
    xlim([0 intrinsics(1)]);
    ylim([0 intrinsics(2)]);
    % 3. get gt
    closest_point_idx = -1;
    seaDepth = bathy.h_final;
    depth = reshape(seaDepth,[], 1);
    groundTruth = nan(size(depth,1), 1);
    r = size(xyz, 1);
%     c = size(xyz, 2);
    real_depth = load(gt_path.path2);
    for i = 1 : r
        if xyz(i, 1) <= 70 % x>90才去计算
            continue;
        end
        ground_idx = -1;
        dis = 1e9;
        for j = 1 : size(objectPointsInCs, 1) %有几个真实数据data行
                ground_idx = i;
                every_dis = sqrt(abs(xyz(i,1)-objectPointsInCs(j,1))^2 + abs(xyz(i,2) + 0 -objectPointsInCs(j,2))^2); %y为50-150所以要加50
                if(every_dis < dis) %如果小，则更新
                    closest_point_idx = j;
                    dis = every_dis;
                end
        end
        if ground_idx ~= -1
            groundTruth(ground_idx) = real_depth(closest_point_idx);
        end

    end

    groundTruth_mat = reshape(groundTruth, size(seaDepth, 2), size(seaDepth,1));
    groundTruth_mat = rot90(groundTruth_mat,3);

    gt = tidyFix(groundTruth_mat, tide);
end