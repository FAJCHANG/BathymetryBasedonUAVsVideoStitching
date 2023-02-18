% video = VideoReader('./DJI_0153.MOV');
% if ~exist('left', 'dir')
%     mkdir('left');
% end
% k = 0;
% while hasFrame(video)
%     k = k + 1;
%     if k > 524
%         frame = readFrame(video);
%         frame = imresize(frame, 0.25);
%         filename = ['./left/' sprintf('%03d',k-524) '.jpg'];
%         imwrite(frame, filename);
%     end
% end
video = VideoReader('./video/VideoTest2.avi');
if ~exist('./results/DownSamples', 'dir')
    mkdir('./results/DownSamples');
end
k = 0;
while hasFrame(video)
    k = k + 1;
    frame = readFrame(video);
    %frame = imresize(frame, 0.5);
    filename = ['./pictures/' sprintf('%03d',k) '.jpg'];
    imwrite(frame, filename);
end