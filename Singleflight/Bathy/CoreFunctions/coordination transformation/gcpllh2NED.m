%GCP每一行为一个经纬高，输入为度形式，不为分秒
%org_llh为原点的经纬度
function gcp_ned = gcpllh2NED(org_llh,gcp_llh)
     gcp_enu =  llh2enu(org_llh,gcp_llh);%这个函数的作用就是将GCP坐标转换到椭球下的笛卡尔坐标系
     %enu->ned的旋转矩阵
     Renu_ned = Euler2Rotate(90,0,180);
     Renu_ned = Renu_ned';
     gcp_enu = gcp_enu';
 for i = 1:size(gcp_enu,2)
     gcp_ned(:,i) = Renu_ned*gcp_enu(:,i);
 end
 
    