function [gcpUV, Expara] = chooseGCP(step3)
%CHOOSEGCP �˴���ʾ�йش˺�����ժҪ
%   �˴���ʾ��ϸ˵��
    %��ʾͼ��
    %%���²���ΪѡȡGCP���Ƶ�
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
    xlabel({ 'U';'����ѡ�˾͵������˳�'});
    ylabel('V');
    hold on;
    %��ʼ������¼�
    x = 1;
    y = 1;
    button = 1;
    UVclick = [];
    unvalid = -99;
    while x<=c && y <= r
        
        title('�밴�������������ѡ��ģʽ');
        pause
        
        title('���ѡ��SCP,�Ҽ�ɾ��SCP');
        [x, y, button] = ginput(1);
        
        if button == 1 && x <= c && y <= r
            x = round(x);
            y = round(y);
            
            title('��������������SCP�Ĳ���');
            
            num = input('������SCP�ı��,ȷ��SCP�������GCP��ͬ:');
            
            text(x+30, y, num2str(num), 'color', 'r', 'fontweight', 'bold', 'fontsize', 15)
            
            sidelength = 10;
            h = rectangle('position', [x-sidelength, y-sidelength, 2*sidelength,2*sidelength],'EdgeColor','r','linewidth',1);

                     
            UVclick = cat(1, UVclick, [num x y]);%����ţ�����㣬���Ƶ㷶Χ�������뾶��������
            
            
            disp(['SCP ' num2str(num)])
            disp(['x:' num2str(x) '  y:' num2str(y)])
            disp(' ')
            
            figure(f1)
            zoom out
        end
        
        if button==3 && (x<=c && y<=r)
            % �ҵ�����ĵ�
            Idx = knnsearch(UVclick(:,2:3),[x y]);

            % Turn the visual display off.
            N=length(UVclick(:,1,1))*2+1; % Total Figure Children (Image+ 1 Text + 1 Marker for each Click)
            f1.Children(1).Children(N-(Idx*2)).Visible='off';   % Turn off Text
            f1.Children(1).Children(N-(Idx*2-1)).Visible='off'; % Turn off Marker
 
            %��ӡɾ����Ϣ
            disp(['ɾ�� SCP ' num2str(UVclick(Idx,1))]);

            % ����Ϊ��Чֵ
            UVclick(Idx,1)=unvalid;
            zoom out
        end
    end
    
    IND=find(UVclick(:,1) ~= unvalid);
    UVsave=UVclick(IND,:);
    [~,ic]=sort(UVsave(:,1));
    UVsave(:,:)=UVsave(ic,:);
    
    %%
    %���²���Ϊ��ʼ�����ⲿ����
    %��ҪĿ����Ѱ�����ֶ�ѡ�еĿ��Ƶ�,�����������꣬�õ���ʼ��ι���
    chosGCPs = [];
    for i = 1:length(UVsave(:,1))
        num = UVsave(i,1);
        chosGCPs(i,:) = gcps(num,:);
    end

    disp('---------->>��ѡ��һ�����Ƶ���Ϊ���������ԭ��,һ�㶼���Ըո�ѡ�ĵ�һ����Ϊԭ��<<----------');
    o_num = input('����һ�����֣�������Ĭ��Ϊ��һ�����Ƶ���Ϊ�������꣺');
    if isempty(o_num)
        o_llh = chosGCPs(1,:);%������������Ϊ�գ���ô���������ԭ��ѡ��Ϊ�ո��ֶ�ѡ��ĵ�һ����
    else
        o_llh = gcps(o_num,:);%��������������ֶ�Ӧ���Ǹ���Ϊ���������
    end
    
    %���涼�ǲο�ʦ�ֵĴ��룬��û��ô����������������
    
    gcpInfo_world = gcpllh2NED(o_llh,chosGCPs);%����ÿ�����Ƶ����������
    gcpInfo_world = gcpInfo_world';
    
%     uavInfo_world = gcpllh2NED(o_llh,step3.world.uav_llh);

    %���ó�ʼ��Σ����׼ȷ��
    extrinsicsInitialGuess = step3.extrinsicsInitialGuess;
    extrinsicsKnownsFlag = step3.extrinsicsKnownsFlag; 
    intrinsics = load(step3.intrinsic);
    intrinsics = intrinsics.intrinsics;
    
    gcpCoord = '�����أ�NED������';
    %�����һ���ṹ��
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
        
    %����������
    for k=1:length(gcp)

        % ��Ŀֻ��֪���߶�����Pz��Ҳ���������Ϣ�����ܵõ�2d->3d,��Ȼ��Ҫ��֪һά����Ϣ�����н⣨x,y,z��
        [xyzReproj(k,:)] = distUV2XYZ(intrinsics,extrinsics,[gcp(k).UVd(1); gcp(k).UVd(2)],'z',gcp(k).z);

        %�������
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
        % ������ɵ�gcp��Ϣ
        h1=plot(gcp(k).UVd(1),gcp(k).UVd(2),'ro','markersize',10,'linewidth',3);
        text(gcp(k).UVd(1)+30,gcp(k).UVd(2),num2str(gcp(k).num),'color','r','fontweight','bold','fontsize',15);

        % ����У����gcp��Ϣ
        h2=plot(UVdReproj(k,1),UVdReproj(k,2),'yo','markersize',10,'linewidth',3);
        text(UVdReproj(k,1)+30,UVdReproj(k,2),num2str(gcp(k).num),'color','y','fontweight','bold','fontsize',15);
    end
    legend([h1 h2],'������ɵ�gcp','�������֮���������gcp');
    for k = 1:length(gcp)
        gcp(k).UVd = UVdReproj(k,:);
    end
    gcpUV = gcp;

end

