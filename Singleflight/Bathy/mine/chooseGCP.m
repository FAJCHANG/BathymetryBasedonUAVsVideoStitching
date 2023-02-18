function [gcpUV, Expara] = chooseGCP(step3)
%CHOOSEGCP 此处显示有关此函数的摘要
%   此处显示详细说明
    %显示图像
    %%以下部分为选取GCP控制点
    gcps = step3.world.gcp_llh;
    
    f1 = figure;
    
    Frames = dir(strcat(step3.imagePath, '/*.jpg'));
    firstFramePath = [Frames(1).folder '\' Frames(1).name];
   
    I = imread(firstFramePath);
    [r,c,~] = size(I);
    colormap(gray);
    imagesc(1:c,1:r,I);
    axis equal;
    xlim([0 c]);
    ylim([0 r]);
    xlabel({ 'U';'不想选了就点这里退出'});
    ylabel('V');
    hold on;
    %初始化鼠标事件
    x = 1;
    y = 1;
    button = 1;
    UVclick = [];
    unvalid = -99;
    while x<=c && y <= r
        
        title('请按下任意键，进入选择模式');
        pause
        
        title('左键选择SCP,右键删除SCP');
        [x, y, button] = ginput(1);
        
        if button == 1 && x <= c && y <= r
            x = round(x);
            y = round(y);
            
            title('请在命令行输入SCP的参数');
            
            num = input('请输入SCP的编号,确保SCP与给定的GCP相同:');
            
            text(x+30, y, num2str(num), 'color', 'r', 'fontweight', 'bold', 'fontsize', 15)
            
            sidelength = 10;
            h = rectangle('position', [x-sidelength, y-sidelength, 2*sidelength,2*sidelength],'EdgeColor','r','linewidth',1);

                     
            UVclick = cat(1, UVclick, [num x y]);%将编号，坐标点，控制点范围，搜索半径保存起来
            
            
            disp(['SCP ' num2str(num)])
            disp(['x:' num2str(x) '  y:' num2str(y)])
            disp(' ')
            
            figure(f1)
            zoom out
        end
        
        if button==3 && (x<=c && y<=r)
            % 找到最近的点
            Idx = knnsearch(UVclick(:,2:3),[x y]);

            % Turn the visual display off.
            N=length(UVclick(:,1,1))*2+1; % Total Figure Children (Image+ 1 Text + 1 Marker for each Click)
            f1.Children(1).Children(N-(Idx*2)).Visible='off';   % Turn off Text
            f1.Children(1).Children(N-(Idx*2-1)).Visible='off'; % Turn off Marker
 
            %打印删除信息
            disp(['删除 SCP ' num2str(UVclick(Idx,1))]);

            % 设置为无效值
            UVclick(Idx,1)=unvalid;
            zoom out
        end
    end
    
    IND=find(UVclick(:,1) ~= unvalid);
    UVsave=UVclick(IND,:);
    [~,ic]=sort(UVsave(:,1));
    UVsave(:,:)=UVsave(ic,:);
    
    %%
    %以下部分为开始计算外部参数
    %首要目标是寻找与手动选中的控制点,计算世界坐标，得到初始外参估计
    chosGCPs = [];
    for i = 1:length(UVsave(:,1))
        num = UVsave(i,1);
        chosGCPs(i,:) = gcps(num,:);
    end

    disp('---------->>请选择一个控制点作为世界坐标的原点,一般都是以刚刚选的第一个点为原点<<----------');
    o_num = input('输入一个数字，不输入默认为第一个控制点作为世界坐标：');
    if isempty(o_num)
        o_llh = chosGCPs(1,:);%如果输入的数字为空，那么世界坐标的原点选择为刚刚手动选择的第一个点
    else
        o_llh = gcps(o_num,:);%否则以输入的数字对应的那个点为世界坐标点
    end
    
    %下面都是参考师兄的代码，还没怎么整理，不过基本能用
    
    gcpInfo_world = gcpllh2NED(o_llh,chosGCPs);%计算每个控制点的世界坐标
    gcpInfo_world = gcpInfo_world';
    
%     uavInfo_world = gcpllh2NED(o_llh,step3.world.uav_llh);

    %设置初始外参（相对准确）
    extrinsicsInitialGuess = step3.extrinsicsInitialGuess;
    extrinsicsKnownsFlag = step3.extrinsicsKnownsFlag; 
    intrinsics = load(step3.intrinsic);
    intrinsics = intrinsics.intrinsics;
    
    gcpCoord = '北东地（NED）：米';
    %打包成一个结构体
    for k=1:size(gcpInfo_world,1)
        gcp(k).UVd = UVsave(k,2:3);
        gcp(k).num = UVsave(k,1);
        gcp(k).Fs = 2;
        gcp(k).x = gcpInfo_world(k,1);
        gcp(k).y = gcpInfo_world(k,2);
        gcp(k).z = gcpInfo_world(k,3);
        gcp(k).WorldCoordSys = gcpCoord;
    end
    x = [gcp(:).x];
    y = [gcp(:).y];
    z = [gcp(:).z];
    
    xyz = [x' y' z'];
    UVd = reshape([gcp(:).UVd],2,length(x))';
    
    [extrinsics,extrinsicsError]= extrinsicsSolver(extrinsicsInitialGuess,extrinsicsKnownsFlag,intrinsics,UVd,xyz);
    
    Expara = extrinsics;
    
    [UVdReproj ] = xyz2DistUV(intrinsics,extrinsics,xyz);
    
    disp(' ')
    disp('Solved Extrinsics and NLinfit Error')
    disp( [' x = ' num2str(extrinsics(1)) ' +- ' num2str(extrinsicsError(1))])
    disp( [' y = ' num2str(extrinsics(2)) ' +- ' num2str(extrinsicsError(2))])
    disp( [' z = ' num2str(extrinsics(3)) ' +- ' num2str(extrinsicsError(3))])
    disp( [' roll = ' num2str(rad2deg(extrinsics(4))) ' +- ' num2str(rad2deg(extrinsicsError(4))) ' degrees'])
    disp( [' pitch = ' num2str(rad2deg(extrinsics(5))) ' +- ' num2str(rad2deg(extrinsicsError(5))) ' degrees'])
    disp( [' yaw = ' num2str(rad2deg(extrinsics(6))) ' +- ' num2str(rad2deg(extrinsicsError(6))) ' degrees'])
        
    %误差分析部分
    for k=1:length(gcp)

        % 单目只有知道尺度因子Pz，也就是深度信息，才能得到2d->3d,不然就要已知一维的信息才能有解（x,y,z）
        [xyzReproj(k,:)] = distUV2XYZ(intrinsics,extrinsics,[gcp(k).UVd(1); gcp(k).UVd(2)],'z',gcp(k).z);

        %计算误差
        gcp(k).xReprojError=xyz(k,1)-xyzReproj(k,1);
        gcp(k).yReprojError=xyz(k,2)-xyzReproj(k,2); 

    end
    %rms=sqrt(nanmean((xyz-xyzReproj).^2));
    
    disp(' ');
    disp('Horizontal GCP Reprojection Error');
    disp( ('GCP Num / X Err /  YErr'));
    
    for k=1:length(gcp)
        disp( ([num2str(gcp(k).num) '/' num2str(gcp(k).xReprojError) '/' num2str(gcp(k).yReprojError) ]));
    end
    
    UVdReproj = reshape(UVdReproj ,[],2);

    disp(UVdReproj);
    f1=figure;
    imshow(I);
    hold on;
    for k=1:length(gcp)
        % 点击生成的gcp信息
        h1=plot(gcp(k).UVd(1),gcp(k).UVd(2),'ro','markersize',10,'linewidth',3);
        text(gcp(k).UVd(1)+30,gcp(k).UVd(2),num2str(gcp(k).num),'color','r','fontweight','bold','fontsize',15);

        % 经过校正的gcp信息
        h2=plot(UVdReproj(k,1),UVdReproj(k,2),'yo','markersize',10,'linewidth',3);
        text(UVdReproj(k,1)+30,UVdReproj(k,2),num2str(gcp(k).num),'color','y','fontweight','bold','fontsize',15);
    end
    legend([h1 h2],'点击生成的gcp','计算外参之后演算出的gcp');
    for k = 1:length(gcp)
        gcp(k).UVd = UVdReproj(k,:);
    end
    gcpUV = gcp;

end

