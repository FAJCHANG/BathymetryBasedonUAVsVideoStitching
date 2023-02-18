function enu=llh2enu(orgllh,llh)%原点，飞机或控制点
for i=1:size(llh,1)
    xyz(i,:)=llh2xyz(llh(i,:));
end
orgxyz = llh2xyz(orgllh);
enu=xyz2enu(xyz,orgxyz);