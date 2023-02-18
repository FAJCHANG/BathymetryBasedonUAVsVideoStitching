function info = calcExtrinsics(step)
%CALCEXTRINSICS �˴���ʾ�йش˺�����ժҪ
%   �˴���ʾ��ϸ˵��


%     dataSample = step4.dataSample;
%     dataSample = dataSample.dataSample;
%     if ~isempty(dataSample)
%         flag = 1;
%     end
    savepath = step.savepath;
    if ~exist([savepath 'gcpsPos'], 'dir')
        mkdir([savepath 'gcpsPos']);
    end

    inputDir = step.inputDir;%�����ļ�·��
    intrinsic = load(step.intrinsic);
    intrinsics = intrinsic.intrinsics;%�ڲ�
    
    gcpUV = step.gcpUV;%��һ֡���Ƶ�����
    Expara = step.Expara;%��һ֡���
    path = step.path;%��Ƶ·��
    
    info(1).gcpUV = gcpUV;%�����һ֡����Ϣ
    info(1).extrinsics = Expara;%�����һ֡����Ϣ
    
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

        H = Pn / Pn_1;%���㵥Ӧ�Ծ���
        
        last_expara = info(nf-1).extrinsics;%������һ֡��Σ�Ŀ����Ϊ�˳�ʼ����˹
        %disp(last_expara);%��εĽṹΪx,y,zΪ�������꣬
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
        extrinsicsInitialGuess = [x y z roll pitch yaw];%����һ֡��λ���Լ��ոս�����Ĵ����̬���
        extrinsicsKnownsFlag = [0,0,0,0,0,0]; 
        
        last_gcp = info(nf-1).gcpUV;%������һ֡��GCP
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
        
        
        
        %�������Լ����
        [extrinsics,extrinsicsError]= extrinsicsSolver(extrinsicsInitialGuess,extrinsicsKnownsFlag,intrinsics,UVd,xyz);
        
        info(nf).extrinsics = extrinsics;%�������
        
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
        title('��ɫ�Ǽ�������Ŀ��Ƶ㣬��ɫ������һ֡�任�����Ŀ��Ƶ�');
        info(nf).gcpUV = last_gcp;    
        hold off
        saveas(gcf, [savepath 'gcpsPos/' num2str(nf, '%03d')], 'jpg');
%         if mod(nf, 15)==0
%             close all;
%         end
        
        
    end

end

