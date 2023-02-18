function [Uo,Vo] = TemplateMatching(Inlast, In, U, V, L, R, U1, V1)
%UNTITLED 此处显示有关此函数的摘要
%   此处显示详细说明
    temp = Inlast(V1-L:V1+L, U1-L:U1+L);%获取上一帧的模板
    roi = In(V-R:V+R, U-R:U+R);%获取当前帧的搜索范围作为匹配的对象
    [m, n] =size(temp);%size的用法，先行后列
    [H, W] = size(roi);

    result = zeros(H-m+1, W-n+1);
    vec_sub = double( temp(:) );
    norm_sub = norm(vec_sub);
    
    
    for i = 1:H-m+1
        for j = 1:W-n+1
            sub_roi = roi(i:i+m-1, j:j+n-1);
            vec = double( sub_roi(:) );
            result(i,j) = vec' * vec_sub / (norm(vec) * norm_sub + eps);
        end
    end
    [iMaxPos, jMaxPos] = find(result == max(result(:)));%i是行y，j是列x
    
%     figure,
%     subplot(121);imshow(temp),title('匹配模板子图像');
%     subplot(122);
%     imshow(roi);
%     title('标记出匹配区域的原图'),
%     hold on
%     plot(jMaxPos,iMaxPos,'*');%绘制最大相关点
%      %用矩形框标记出匹配区域
%     plot([jMaxPos,jMaxPos+n-1],[iMaxPos,iMaxPos]);
%     plot([jMaxPos+n-1,jMaxPos+n-1],[iMaxPos,iMaxPos+m-1]);
%     plot([jMaxPos,jMaxPos+n-1],[iMaxPos+m-1,iMaxPos+m-1]);
%     plot([jMaxPos,jMaxPos],[iMaxPos,iMaxPos+m-1]);

    Uo = jMaxPos + U - R + L;
    Vo = iMaxPos + V - R + L;
end

