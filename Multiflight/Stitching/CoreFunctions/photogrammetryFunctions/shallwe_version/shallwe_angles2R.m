%�ú������ڼ�����ת����Rec = Ref*Rfc
%RecΪ��ʵ����ϵ������������ת����
%RefΪ��ʵ���굽�����������ת����
%RfcΪ�������굽����������ת����

%����ĽǶ�Ϊ�����ŷ����,��EXIF��Ϣ�õ���Ҫ�ú����һ��exif�����ÿ����ת�ǵ����壬����NEDת��CAMERA����ϵ����ת����
function [R] = shallwe_angles2R(roll,pitch,yaw,mode)
    if nargin <= 3
        mode = 1;
    end
    
    if mode == 1  %��̬ŷ������ת����
        R = varible_rm(pi/2+roll,pitch,pi/2+yaw);
    else
        R = static_rm(roll,pi/2+pitch,pi/2+yaw);
    end

end


function R = static_rm(roll,pitch,yaw)
    c1 = cos(roll);
    c2 = cos(pitch);
    c3 = cos(yaw);
    s1 = sin(roll);
    s2 = sin(pitch);
    s3 = sin(yaw);
    %��̬ŷ����ZYX��ת��������,Rz*Ry*Rx,��ת˳��Ϊ��Z->Y->X
    R = [
            c3*c2, s3*c1+c3*s2*s1, s3*s1-c3*s2*c1;
            -s3*c2 , c3*c1-s3*s2*s1 , c3*s1+s3*s2*c1;
            s2 , -c2*s1, c2*c1
        ];

end

function R = varible_rm(roll,pitch,yaw)
    c1 = cos(roll);
    c2 = cos(pitch);
    c3 = cos(yaw);
    s1 = sin(roll);
    s2 = sin(pitch);
    s3 = sin(yaw);
    
    %��̬ŷ����ZYX��ת,��������ת˳��Ϊ��Z->Y->X,Rx*Ry*Rz
    Rz = [c3, -s3, 0;
        s3, c3, 0;
        0, 0, 1];
    Ry = [c2, 0, s2;
        0, 1, 0;
        -s2, 0, c2];
    Rx = [1, 0, 0;
        0, c1, -s1;
        0, s1, c1];
    R = Rz*Ry*Rx;
    R = R';
%     R = [
%                  c2*c3,                  c2*s3,              -s2;
%         s1*s2*c3-c1*s3,         s1*s2*s3+c1*c3,            s1*c2;
%         c1*c3*s2+s1*s3,         c1*s2*s3-s1*c3,            c1*c2
%     ];

end