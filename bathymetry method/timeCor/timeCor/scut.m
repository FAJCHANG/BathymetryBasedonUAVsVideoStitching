%% 本脚本用于scut毕业论文画图

%%
% load('data_final.mat');
% load('data_struct.mat');
addpath(genpath('algoV1'));
addpath(genpath('algoV2'));
addpath(genpath('common'));

%% 傅里叶变换频域分布图
eval('phantom4');
addpath('./algoV2/makeTimeStack');
data_lowpass = lowPassFliter(data, params);
Fs = params.fs;
N = params.N;
n = 0:N-1;
t = n / Fs;
% s1 = data_final(:, 15100); % 
% s2 = data_final(:, 15201); % 140

% xyz(14191, :);

% s2 = data_final(:, 15201);
% xyz(15201, :);
s1 = data(:, 15100); % 
s2 = data_lowpass(:, 15201); %

G1 = fft(detrend(double(s1)))';
G2 = fft(double(s2))';
df = Fs / N; 
dt = 1 / Fs;
f = [0 : df : 1 / (2 * dt) - df];
% figure
% plot(f, abs(G1(:, 1 : length(f))), 'b');

% title('滤波信号频域图');

% plot_args.point_end = 100;
% plot_args.point_begin = 1;
% point_begin = 1;
% plot(t(plot_args.point_begin : plot_args.point_end), s1(plot_args.point_begin :plot_args.point_end)', 'g');
% hold on;
% plot(t(plot_args.point_begin : plot_args.point_end), s2(plot_args.point_begin :plot_args.point_end)', 'r');
% set(gca,'XTick',[0:10:100]);
% set(gca,'YTick',[0:50:600]);
% 
% xlabel('time(s)');
% ylabel('pixel intensity');
% legend('149m','150m');


% title('pixel intensity fluctuation trend');

s3 = data(:, 29128);
G3 = fft(detrend(double(s3)))';

figure
plot(f, abs(G1(:, 1 : length(f))), 'color', 'k','linewidth', 5);
% title('Frequency distribution of signal after main frequency extraction');


hold on;
s4 = data_final(:, 29128); % 
G4 = fft(detrend(double(s4)))';
df = Fs / N; 
dt = 1 / Fs;
f = [0 : df : 1 / (2 * dt) - df];
plot(f, abs(G2(:, 1 : length(f))),'color','r', 'linewidth', 2);
xlabel('频率(Hz)');
ylabel('傅里叶变换幅值');
set(gca,'FontSize',15);
legend('原信号','滤波信号');

close all;

%% 时间堆栈画图（timestack.jpg）
eval('phantom4');
img_path = params.final_path;
time_stack = plotTimeStack(img_path, 100);
close all;
imagesc(time_stack);
colormap(gray)


set(gca,'XTick',[0:100:params.N]);
set(gca,'xticklabel',[0 : 100 : params.N] / params.fs);
set(gca,'YTick',[0:50:params.N]);
set(gca,'yticklabel',[0: 50: params.N] .* 0.5);


set(gca,'YDir','reverse'); 
set(gca,'FontSize',30); 
% ylim([100 400])
xlabel('传播时间(s)','FontSize',30);
ylabel('跨岸距离(m)','FontSize',30);
%% 找到最佳估计范围(timeCor_range.jpg)
% part1
station_str = 'phantom4';
% 执行参数注入
eval(station_str);
x = xyz(:, 1);
y = xyz(:, 2);
z = xyz(:, 3);
longshore = 100;
one_long_shore_id = find(y == longshore);
cross_shore_x = x(one_long_shore_id, :);
cor_num = 60;
one_long_shore_data = data_final(:, one_long_shore_id);
cor = nan(cor_num, max(cross_shore_x) - min(cross_shore_x) + 1);
params.DEBUG = 1;
for cross_shore = min(cross_shore_x) : params.dist : max(cross_shore_x)

% for cross_shore = 150 : 150
    ref_id = find(cross_shore_x == cross_shore); % 找到所在的索引
    ref = one_long_shore_data(:, ref_id);
    data_set = one_long_shore_data(:, 1 : (ref_id - 1));
%         range_id = getSuitRange1(ref, ref_id, data_set, params); % version1：获取互相关最大的那个值的索引，称为range_id，没有对信号进行筛选而得到的
    [range_id, cor(:, cross_shore - min(cross_shore_x) + 1)] = getSuitRange2(ref(1: end - params.fix_time / params.dt), data_set(1 + params.fix_time / params.dt : end, :), params);
    [mid_id, speed, f] = getMidSpeedFreq(ref, ref_id, data_set, params, range_id);% 获取最相关的一个
    disp(['progress:' num2str((cross_shore - min(cross_shore_x)) / (max(cross_shore_x) - min(cross_shore_x))* 100) '% completed']);
end

%% part2
plot_cor = flipud(cor);
plot_cor(:, 1: 61) = 0;
cmap = colormap( 'jet' );
colormap( flipud( cmap ) );
pcolor(plot_cor);
shading flat;
caxis([-1 1]); %设置深度的显示范围
axis equal;
axis tight;
xlabel('跨岸距离(m)','FontSize',30);
ylabel('与目标点距离(m)','FontSize',30);
cor_val = colorbar('peer', gca);
set(gca,'ydir','reverse');
% set(gca,'yticklabel',[60: -1 : 1]);
set(get(cor_val,'title'),'string', '互相关系数');

hold on;
% 找到每行最大值
for i = 61 : size(cor, 2)
    cor_struct.col = cor(:, i);
    [~, cor_struct.max_id] = max(cor_struct.col);
    cor_struct.max_point(i - 60, :) = [cor_struct.max_id, i];
end

plot(cor_struct.max_point(:,2), size(cor, 1) - cor_struct.max_point(:,1),'LineWidth', 2, 'color','w');
set(gca,'FontSize',30); 
%% 高斯平滑滤波（在ProcessFromVideo/paperImage.m中）
%% 临近像素点
%% 对比图
close all;
imagesc(time_stack);
colormap(gray)



set(gca,'XTick',[0:100:params.N]);
set(gca,'xticklabel',[0 : 100 : params.N] / params.fs);
set(gca,'YTick',[0:50:params.N]);
set(gca,'yticklabel',[0: 50: params.N] .* 0.5);

line([0, params.N], [254, 254],'Color','g','LineWidth', 2);
line([0, params.N], [298, 298],'Color','b','LineWidth', 2);
line([0, params.N], [300, 300],'Color','red','LineWidth', 2);

set(gca,'YDir','reverse'); 
% ylim([100 400])
set(gca,'FontSize',30); 
xlabel('传播时间(s)','FontSize',30);
ylabel('跨岸距离(m)','FontSize',30);
legend('134m','149m','150m','FontSize',30);

%% 对比图2

figure
Fs = params.fs;
N = params.N;
n = 0:N-1;
t = n / Fs;
s1 = data_final(:, 15100); % 
% xyz(14191, :);
% s2 = data_final(:, 15201); % 140
s2 = data_final(:, 15201);

figure

plot_args.point_end = 100;
plot_args.point_begin = 1;
point_begin = 1;
plot(t(plot_args.point_begin : plot_args.point_end), s1(plot_args.point_begin :plot_args.point_end)','color', 'b','linewidth',3);
hold on;
plot(t(plot_args.point_begin : plot_args.point_end), s2(plot_args.point_begin :plot_args.point_end)' ,'color', 'r','linewidth',3);
% set(gca,'XTick',[0:10:100]);
% set(gca,'YTick',[0:50:600]);

xlabel('时间(s)','FontSize',30);
ylabel('像素强度','FontSize',30);
legend('149m','150m','FontSize',30);
set(gca,'FontSize',30); 


%% 互功率谱分析完的频域分布图

Fs = 2;
N = params.N;
n = 0:N-1;
t = n / Fs;
s3 = data(:, 29128);
G3 = fft(detrend(double(s3)))';

figure
plot(f, abs(G3(:, 1 : length(f))), 'color', 'k','linewidth', 2);
% title('Frequency distribution of signal after main frequency extraction');


hold on;
s4 = data_final(:, 29128); % 
G4 = fft(detrend(double(s4)))';
df = Fs / N; 
dt = 1 / Fs;
f = [0 : df : 1 / (2 * dt) - df];
plot(f, abs(G4(:, 1 : length(f))),'color','r', 'linewidth', 2);
xlabel('频率(Hz)');
ylabel('傅里叶变换幅值');
set(gca,'FontSize',30);
legend('原信号频率分布','互功率谱分析后保留频率分布');

%% 高斯低通滤波器
%


