function info = calcExtrinsics(step)
%CALCEXTRINSICS 此处显示有关此函数的摘要
%   此处显示详细说明


%     dataSample = step4.dataSample;
%     dataSample = dataSample.dataSample;
%     if ~isempty(dataSample)
%         flag = 1;
%     end
    savepath = step.savepath;
    if ~exist([savepath 'gcpsPos'], 'dir')
        mkdir([savepath 'gcpsPos']);
    end

    inputDir = step.inputDir;%输入文件路径
    intrinsic = load(step.intrinsic);
    intrinsics = intrinsic.intrinsics;%内参
    
    gcpUV = step.gcpUV;%第一帧控制点坐标
    Expara = step.Expara;%第一帧外参
    path = step.path;%视频路径
    
    info(1).gcpUV = gcpUV;%存入第一帧的信息
    info(1).extrinsics = Expara;%存入第一帧的信息
    
    fx=intrinsics(5);
    fy=intrinsics(6);
    c0U=intrinsics(3);
    c0V=intrinsics(4);
    K = [fx 0 c0U;
        0 fy c0V;
        0  0 1];
       
    nFrames = length(dir(strcat(inputDir, '/*.jpg')));
    L = dir(inputDir);
    L = L(3:end);
    for nf = 1:nFrames
        if nf == 1
            figure(1);
            I = imread(strcat(inputDir, L(nf).name));
            imshow(I)
            hold on;
            for k=1:length(gcpUV)
                h1=plot(gcpUV(k).UVd(1),gcpUV(k).UVd(2),'ro','markersize',10,'linewidth',3);
            end
            saveas(gcf, [savepath 'gcpsPos/' num2str(nf, '%03d')], 'jpg');
            continue;
        end
        I = imread(strcat(inputDir, L(nf).name));
        Pn = squeeze(path(nf,1,1,:,:));
        Pn_1 = squeeze(path(nf-1,1,1,:,:));

        H = Pn / Pn_1;%计算单应性矩阵
        
        last_expara = info(nf-1).extrinsics;%加载上一帧外参，目的是为了初始化高斯
        %disp(last_expara);%外参的结构为x,y,z为世界坐标，
        x=last_expara(1);
        y=last_expara(2);
        z=last_expara(3);
        
        roll = last_expara(4);
        pitch = last_expara(5);
        yaw = last_expara(6);
        R1 = shallwe_angles2R(roll,pitch,yaw);
        R2 = K \ H * K * R1;
        [yaw,pitch,roll]=dcm2angle(R2, 'ZYX');
        roll = roll - pi/2;
        yaw = yaw - pi/2;
        
%         extrinsicsInitialGuess = [dataSample(nf,:) roll pitch yaw];
        extrinsicsInitialGuess = [x y z roll pitch yaw];%用上一帧的位置以及刚刚解算出的大概姿态求解
        extrinsicsKnownsFlag = [0,0,0,0,0,0]; 
        
        last_gcp = info(nf-1).gcpUV;%加载上一帧的GCP
        x = [last_gcp(:).x];
        y = [last_gcp(:).y];
        z = [last_gcp(:).z];
        xyz = [x' y' z'];
        
%         UVd = reshape([last_gcp(:).UVd],2,length(x));
%         UVd = [UVd;ones(1,length(x))];
%         UVd = H * UVd;
%         UVd = UVd./UVd(3,:);
%         UVd = UVd(1:2, 1:length(x))';


        first_gcp = info(1).gcpUV;
        UVd = reshape([first_gcp(:).UVd],2,length(x));
        UVd = [UVd;ones(1,length(x))];
        UVd = Pn * UVd;
        UVd = UVd./UVd(3,:);
        UVd = UVd(1:2, 1:length(x))';
        
        
        
        %求解外参以及误差
        [extrinsics,extrinsicsError]= extrinsicsSolver(extrinsicsInitialGuess,extrinsicsKnownsFlag,intrinsics,UVd,xyz);
        
        info(nf).extrinsics = extrinsics;%保存外参
        
        [UVdReproj] = xyz2DistUV(intrinsics,extrinsics,xyz);
        
        
        UVdReproj = reshape(UVdReproj,[],2);
        figure(1);
        imshow(I)
        hold on
        for d = 1:length(UVdReproj)
            last_gcp(d).UVd = UVdReproj(d,:);
            h1=plot(UVd(d,1),UVd(d,2),'ro','markersize',10,'linewidth',3);
            h2=plot(UVdReproj(d,1),UVdReproj(d,2),'yo','markersize',10,'linewidth',3);
        end
        title('黄色是计算出来的控制点，红色是由上一帧变换过来的控制点');
        info(nf).gcpUV = last_gcp;    
        hold off
        saveas(gcf, [savepath 'gcpsPos/' num2str(nf, '%03d')], 'jpg');
%         if mod(nf, 15)==0
%             close all;
%         end
        
        
    end

end

