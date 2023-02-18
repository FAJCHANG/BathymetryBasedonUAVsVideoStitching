function [pixelImg] = getPixelImage(step, info)
%GETPIXELIMAGE �˴���ʾ�йش˺�����ժҪ
%   �˴���ʾ��ϸ˵��
    
    gausskernel = 50;

    inputdir = step.inputdir;
    savepath = step.savepath;
    filt_path = step.filt_path;
    
    localAngle = step.local_angle;
    localOrigin = step.local_origin;
    
    intrinsics = load(step.intrinsic);
    intrinsics = intrinsics.intrinsics;
    
    zFixedCam = {};
    
    gcps = step.gcps;
    o_llh = gcps(1,:);
    gcpInfo_world = gcpllh2NED(o_llh,gcps);
    
    idxdy = step.pixel_resolution;%�ֱ��� resolution
    ixlim = [0,100];%�纣���� cross-shore 
    iylim = [0,100];%��̲����  longshore
    iz = 0;
    
    inputStruct.roi_x = [0,100];
    inputStruct.roi_y = [0,100];
    inputStruct.dx = 0.5;
    inputStruct.dy = 0.5;

    inputStruct.x_dx = 0.5;
    inputStruct.x_oy = 0;
    inputStruct.x_rag = [0,100];

    inputStruct.y_dy = 0.5;
    inputStruct.y_ox = 0;
    inputStruct.y_rag = [0,100];
    
    
    Lo = dir(inputdir);
    Lo = Lo(3:end);
    
    worldCoord = info(1).gcpUV(1).WorldCoordSys;
    [iX,iY]=meshgrid([ixlim(1):idxdy:ixlim(2)],[iylim(1):idxdy:iylim(2)]);
    iZ=iX*.0+iz;
    localX=iX;
    localY=iY;
    localZ=iZ;
    [X,Y]=localTransformEquiGrid(localOrigin,localAngle,0,iX,iY);
    Z=X*.0+iz; 
    save([savepath '/GRID_roiInfo0' ], 'X', 'Y', 'Z', 'worldCoord', 'localAngle','localOrigin','localX','localY','localZ');


    if length(Lo) == length(info)
    
        for id = 1:length(info)
            temp_intrinsics = intrinsics;
            the_gcp = info(id).gcpUV;
            the_extrinsic = info(id).extrinsics;
            

            imagepath = strcat(inputdir, Lo(id).name);
            Io = imread(imagepath);
            
            If = gaussfilter(Io,gausskernel);
            imwrite(If, [filt_path 'filt_' num2str(id, '%04d') '.jpg']);
            temp_size = size(If);
            temp_intrinsics(1) = temp_size(2);
            temp_intrinsics(2) = temp_size(1);

            
            [iX,iY]=meshgrid([ixlim(1):idxdy:ixlim(2)],[iylim(1):idxdy:iylim(2)]);
            iZ=iX*0+iz;
            
            
            pixInst(1).type='Grid';
            pixInst(1).dx =inputStruct.dx;
            pixInst(1).dy =inputStruct.dy;
            pixInst(1).xlim =inputStruct.x_rag;
            pixInst(1).ylim =inputStruct.y_rag;
            pixInst(1).z={}; 
            
            pixInst(2).type='yTransect';
            pixInst(2).x= inputStruct.y_ox; % y������Ӧx�ĳ�ʼλ��
            pixInst(2).ylim=inputStruct.y_rag;
            pixInst(2).dy =inputStruct.y_dy; %���طֱ���
            pixInst(2).z ={};
            
            pixInst(3).type='xTransect';
            pixInst(3).y= inputStruct.x_oy;
            pixInst(3).xlim=inputStruct.x_rag;
            pixInst(3).dx =inputStruct.x_dx;
            pixInst(3).z ={}; 
            
            X = iX;
            Y = iY;
            Z = iZ;
            
            the_extrinsic = localTransformExtrinsics(localOrigin,localAngle,1,the_extrinsic);
            [pixInst]=pixInstPrepXYZ(pixInst);
            
            for p=1:length(pixInst)
                if isempty(pixInst(p).z)==1
                    if isempty(zFixedCam)==1 % If a Time-varying z is not specified 
                        pixInst(p).Z=interp2(X,Y,Z,pixInst(p).X,pixInst(p).Y); % ���������zֵ��ѡ���������в�ֵ
                    end

                    if isempty(zFixedCam)==0 % If a time varying z is specified
                        pixInst(p).Z=pixInst(p).X.*0+zFixedCam(1); % �����ȷzֵ�Ǻ����Ķ�Ϊ��ֵ
                    end

                end
            end
            
            
            for p=1:length(pixInst)
                % Check if a time varying Z was specified. If not, wil just use constant Z
                % specified or interpolated from grid in Section 4 and 6 respectively.
                if isempty(pixInst(p).z)==1  % �����ʱ��zֵ��Ϊ��ʱ
                    if isempty(zFixedCam)==0 % ���ʱ�̶�����Ļ�z���ڹ̶�ֵ����������̶�ֵ����
                        pixInst(p).Z=pixInst(p).X.*0+zFixedCam(id); % Assign First Value for First Image, spatially constant elevation
                    end
                end
                
%                 [Iorgb]= imageRectifier(Io,intrinsics,the_extrinsic,pixInst(p).X,pixInst(p).Y,pixInst(p).Z,0); 
%                 [Iogray]=rgb2gray(Iorgb);
%                 pixImg(p).Igray = Iogray;
%                 pixImg(p).Irgb = Iorgb;
                [Ifgray]= imageRectifier(If,temp_intrinsics,the_extrinsic,pixInst(p).X,pixInst(p).Y,pixInst(p).Z,0); 
%                 [temp]= imageRectifier(It,intrinsics,the_extrinsic,pixInst(p).X,pixInst(p).Y,pixInst(p).Z,0); 
%                 subplot(2,2,1);imshow(If);
%                 subplot(2,2,2);imshow(Ifgray);
%                 subplot(2,2,3);imshow(It);
%                 subplot(2,2,4);imshow(temp);  
%                 imshow(Ifgray);
                pixImg(p).Ifilt = Ifgray;
            end
            pixelImg(id).info = pixImg;
            disp([ 'Frame ' num2str(id) ' out of ' num2str(length(info)) ' completed. ' num2str(round(id/length(info)*1000)/10) '%']) 
            img = pixImg.Ifilt;
            img = rot90(img(:,:,:),3);
            imshow(img);
            hold on
            imwrite(img,[step.saverotpath 'finalOrth_'  num2str(id, '%04d') '.jpg'],'jpg');
            hold off
        end
        save([savepath 'pixelImg_' num2str(gausskernel) ], 'pixelImg');
            
    else
        disp('ͼƬ��������Ƶ��������Ŀ��һ��/The number of pictures is different from the number of extrinsics parameters of control points');
    end
    
    disp('Finish');

end
% Gaussian filter
function [image_result] =gaussfilter(image_orign,D0) 
    %GULS ��˹��ͨ�˲���
    % D0Ϊ����Ƶ�ʵģ��൱�������ڸ���Ҷ��ͼ�İ뾶ֵ��
    if (ndims(image_orign) == 3)
    %�ж϶����ͼƬ�Ƿ�Ϊ�Ҷ�ͼ�����������ת��Ϊ�Ҷ�ͼ���������������
    image_2zhi = rgb2gray(image_orign);
    else 
    image_2zhi = image_orign;
    end
    image_fft = fft2(image_2zhi);%�ø���Ҷ�任��ͼ��ӿռ���ת��ΪƵ����
    image_fftshift = fftshift(image_fft);
    %����Ƶ�ʳɷ֣�����ԭ�㣩�任������ҶƵ��ͼ����
    [width,high] = size(image_2zhi);
    D = zeros(width,high);
    %����һ��width�У�high�����飬���ڱ�������ص㵽����Ҷ�任���ĵľ���
    for i=1:width
    for j=1:high
        D(i,j) = sqrt((i-width/2)^2+(j-high/2)^2);
    %���ص㣨i,j��������Ҷ�任���ĵľ���
        H(i,j) = exp(-1/2*(D(i,j).^2)/(D0*D0));
    %��˹��ͨ�˲�����
        image_fftshift(i,j)= H(i,j)*image_fftshift(i,j);
    %���˲������������ص㱣�浽��Ӧ����
    end
    end
    image_result = ifftshift(image_fftshift);%��ԭ�㷴�任��ԭʼλ��

    image_result = uint8(real(ifft2(image_result)));
end
