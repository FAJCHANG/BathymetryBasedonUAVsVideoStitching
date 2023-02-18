function downSampleFromVideo(step)

    %如果不设置采样频率和采样帧时，默认为2Hz和全部帧转换
    
    videoPath = step.videoPath;
    savePath = step.savePath;
    filterPath = step.filterPath;
    % determine downsample frequence. 
    
    v = VideoReader(videoPath);
    
    if isfield(step,'fs') && ~isempty(step.fs)
        fs = step.fs;
    else
        fs = 2;
    end
    
    % determine used video range 
    if isfield(step,'videoRange') && ~isempty(step.videoRange)
        videoRange = step.videoRange;
    else
        videoRange = [0 v.Duration];
    end
    
    % determine downsample image pixel resolution.
    if isfield(step,'pixel') && ~isempty(step.pixel)
        pixel = step.pixel;
        if length(pixel) < 2
            warning('uncorrect pixel resolution setting');
            return;
        end
    else 
        pixel = [];
    end
    
    to = datenum(2021,01,21,7,30,0); %第一帧的年月日时分秒，不知道也没事直接设置为0
    
    SaveName = 'downSample';
       
    % 初始化时间
    if to==datenum(0,0,0,0,0,0) % 如果不知道时间默认为0
        to=0;
    else % if to known
        to=(to-datenum(1970,1,1)) * 24 * 3600; % 从unix系统默认诞生时间开始算的秒数
    end

    % 初始化循环
    k=1;
    count=1;
    numFrames = v.Duration .* v.FrameRate;  % 视频总帧数

    while k <= numFrames

        I = read(v, k);
        if k == 1
            vto=v.CurrentTime;%单位为秒
        end

        t = v.CurrentTime;
        
        if t < videoRange(1)
            k = k + round(v.FrameRate ./ fs);
            disp(['waiting for extraction:' num2str(videoRange(1)-t) 's']);
            continue;
        elseif t > videoRange(2)
            disp('Out of specific video range');
            break;
        end

        ts= (t-vto)+to; % ts为提取帧对应的秒数,单位为s
        %Because of the way Matlab defines time. 
        if k == numFrames
            ts=ts+1./v.FrameRate;
        end

        % rounded time to ensure filename completation.
        ts=round(ts.*1000);
        
        % resize image.
        if ~isempty(pixel) 
            I = imresize(I, pixel); % pixel is 1*2 vector:[numrows numcols]
        end
            
        % save
        imwrite(I,[savePath SaveName '_' num2str(ts) '.jpg']); %可以考虑放到最后去灰度化
        I_filter = gaussfilter(I, step.d0);
        imwrite(I_filter,[filterPath SaveName '_' num2str(ts) '.jpg']);%可以考虑放到最后去灰度

        % 显示进度，非常叼的机制
        disp([ num2str( (t-videoRange(1))./(videoRange(2)- videoRange(1))*100) '% Extraction Complete'])

        % 得到下一帧的索引
        k=k+round(v.FrameRate./fs);

        % 储存时间信息
        T(count)=ts/1000; %转化为秒
        count=count+1;

    end
    
    
    
    %显示转换完成的信息
    disp(' ');
    disp(['原始视频帧率: ' num2str(v.FrameRate) ' fps'])
    disp(['指定视频帧率: ' num2str(fs) ' fps']);
    disp(['指定提取图片的时间间隔: ' num2str(1./fs) ' s']);
    disp(['实际平均时间间隔: ' num2str(nanmean(diff(T(1:(end-1))))) ' s']);
% 	disp(['STD of actual dt: ' num2str(sqrt(var(diff(T(1:(end-1))),'omitnan'))) ' s']);

end


function [image_result] =gaussfilter(image_orign,D0)

    %GULS 高斯低通滤波器

    % D0为截至频率的（相当于设置在傅里叶谱图的半径值）

    if (ndims(image_orign) == 3)

    %判断读入的图片是否为灰度图，如果不是则转换为灰度图，如果是则不做操作

    image_2zhi = rgb2gray(image_orign);

    else 

    image_2zhi = image_orign;

    end

    image_fft = fft2(image_2zhi);%用傅里叶变换将图象从空间域转换为频率域

    image_fftshift = fftshift(image_fft);

    %将零频率成分（坐标原点）变换到傅里叶频谱图中心

    [width,high] = size(image_2zhi);

    D = zeros(width,high);

    %创建一个width行，high列数组，用于保存各像素点到傅里叶变换中心的距离

    for i=1:width

    for j=1:high

        D(i,j) = sqrt((i-width/2)^2+(j-high/2)^2);

    %像素点（i,j）到傅里叶变换中心的距离

        H(i,j) = exp(-1/2*(D(i,j).^2)/(D0*D0));

    %高斯低通滤波函数

        image_fftshift(i,j)= H(i,j)*image_fftshift(i,j);

    %将滤波器处理后的像素点保存到对应矩阵

    end

    end

    image_result = ifftshift(image_fftshift);%将原点反变换回原始位置

    image_result = uint8(real(ifft2(image_result)));
end

