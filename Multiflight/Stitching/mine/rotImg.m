function rotImg(step)
%ROTIMG 此处显示有关此函数的摘要
%   此处显示详细说明
    pixel_path = step.pixelpath;
    tmp1 = load(pixel_path);
    pixImg = tmp1.pixelImg;
    
    savePath = step.savePath;
    for i = 1:length(pixImg)
        img = pixImg(i).info.Ifilt;
        img = rot90(img(:,:,:),3);
        imwrite(img,[savePath 'finalOrth_'  num2str(i, '%04d') '.jpg'],'jpg');
        disp([num2str(i/length(pixImg)*100) '% completed']);
    end
    
end

