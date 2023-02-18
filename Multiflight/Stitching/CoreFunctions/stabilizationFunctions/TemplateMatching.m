function [Uo,Vo] = TemplateMatching(Inlast, In, U, V, L, R, U1, V1)
%UNTITLED �˴���ʾ�йش˺�����ժҪ
%   �˴���ʾ��ϸ˵��
    temp = Inlast(V1-L:V1+L, U1-L:U1+L);%��ȡ��һ֡��ģ��
    roi = In(V-R:V+R, U-R:U+R);%��ȡ��ǰ֡��������Χ��Ϊƥ��Ķ���
    [m, n] =size(temp);%size���÷������к���
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
    [iMaxPos, jMaxPos] = find(result == max(result(:)));%i����y��j����x
    
%     figure,
%     subplot(121);imshow(temp),title('ƥ��ģ����ͼ��');
%     subplot(122);
%     imshow(roi);
%     title('��ǳ�ƥ�������ԭͼ'),
%     hold on
%     plot(jMaxPos,iMaxPos,'*');%���������ص�
%      %�þ��ο��ǳ�ƥ������
%     plot([jMaxPos,jMaxPos+n-1],[iMaxPos,iMaxPos]);
%     plot([jMaxPos+n-1,jMaxPos+n-1],[iMaxPos,iMaxPos+m-1]);
%     plot([jMaxPos,jMaxPos+n-1],[iMaxPos+m-1,iMaxPos+m-1]);
%     plot([jMaxPos,jMaxPos],[iMaxPos,iMaxPos+m-1]);

    Uo = jMaxPos + U - R + L;
    Vo = iMaxPos + V - R + L;
end

