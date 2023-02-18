function chooseROIshow(debug)
    %先将所有参数赋值
    info = debug.info;
    intrinsics = debug.intrinsics;
    intrinsics = intrinsics.intrinsics;
    ix = debug.ix;
    iy = debug.iy;
    resolution = debug.resolution;
    inputdir = debug.inputdir;
    savepath = debug.savepath;
    
    gcp_llh = debug.gcp_llh;
    o_llh = debug.o_llh;
    index = debug.index;
    
    
    imagePoints = [];
    gcpUV = info(index).gcpUV;
    choose_gcp_llh = [];
    for i = 1:length(gcpUV)
        imagePoints = [imagePoints; gcpUV(i).UVd];
        num = gcpUV(i).num;
        choose_gcp_llh = [choose_gcp_llh; gcp_llh(num,:)];
    end
    
    
    
    objectPoints = gcpllh2NED(o_llh,choose_gcp_llh);
    objectPoints = objectPoints';
    cameraMatrix.mat = [intrinsics(5) 0 intrinsics(3); 0 intrinsics(6) intrinsics(4); 0 0 1];
    cameraMatrix.dist = intrinsics(7:11);
    

    Frames = dir(strcat(inputdir, '/*.jpg'));
    indexFramePath = [Frames(index).folder '\' Frames(index).name];
    src = imread(indexFramePath);
%     tmp = imread('../results/resMat/ROIrange.jpg');
    tmp = src;
    
    Rotate_ned2cs = Euler2Rotate(debug.local_angle,0,0);
    Rotate_ned2cs = Rotate_ned2cs';
    objectPointsInCs = Rotate_ned2cs*objectPoints';
    objectPointsInCs = objectPointsInCs';
    
    expara = info(index).extrinsics;
    Re_c = expara(4:6);
    Re_c = shallwe_angles2R(Re_c(1), Re_c(2), Re_c(3));
    tvec = -Re_c*expara(1:3)';
    
    crossRange = ix;
    longRange = iy;
    seaLevel = 0;
    pixelInterval = resolution;
    
    testWorldCor1 = Rotate_ned2cs' * [crossRange(1);longRange(1);seaLevel];
    testWorldCor2 = Rotate_ned2cs' * [crossRange(2);longRange(1);seaLevel];
    testWorldCor3 = Rotate_ned2cs' * [crossRange(2);longRange(2);seaLevel];
    testWorldCor4 = Rotate_ned2cs' * [crossRange(1);longRange(2);seaLevel];
    
    testImageCor1 = world2image(cameraMatrix,Re_c,tvec,testWorldCor1);
    testImageCor2 = world2image(cameraMatrix,Re_c,tvec,testWorldCor2);
    testImageCor3 = world2image(cameraMatrix,Re_c,tvec,testWorldCor3);
    testImageCor4 = world2image(cameraMatrix,Re_c,tvec,testWorldCor4);
    
    clr = 'red';
    
    tmp = insertShape(tmp,'Line',[testImageCor1(1) testImageCor1(2) testImageCor2(1) testImageCor2(2)],'LineWidth',4,'Color',clr);
    tmp = insertShape(tmp,'Line',[testImageCor2(1) testImageCor2(2) testImageCor3(1) testImageCor3(2)],'LineWidth',4,'Color',clr);
    tmp = insertShape(tmp,'Line',[testImageCor3(1) testImageCor3(2) testImageCor4(1) testImageCor4(2)],'LineWidth',4,'Color',clr);
    tmp = insertShape(tmp,'Line',[testImageCor4(1) testImageCor4(2) testImageCor1(1) testImageCor1(2)],'LineWidth',4,'Color',clr);
    
    
    
    h = figure(1);
    imshow(tmp);
    disp('save roi to mat_savepath');
    print(h,'./ROIrange.jpg','-r300','-djpeg');
    
%     resImage = downSample(src,crossRange,longRange,pixelInterval,pixelInterval,cameraMatrix,Re_c,tvec); %下采样函数
%     figure(2);
%     imshow(resImage);
    

end