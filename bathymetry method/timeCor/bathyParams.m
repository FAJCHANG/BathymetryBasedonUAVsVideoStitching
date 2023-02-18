%%% ��������
params.station_str = 'phantom4';    % �ز���Դƽ̨���� Material source platform name

params.data_save_path = '.../Multiflight/Stitching/results/input/'; % ���ݴ����ı���·�� Save path after data processing
params.img_path = '.../Multiflight/Stitching/results/filt/';   % �˲����ͼ Filtered graph
params.final_path = '.../Multiflight/Stitching/results/orthImg/'; % ����任��ͼ Images of Orthotropic Transformation

params.xy_min_max = [0 100 0 100];  % local����ϵ xy��ķ�Χ��12��Ԫ����x�ᣬ34��y��  The range of the xy axis of the local coordinate system. 1 2 elements are the x axis and 3 4 are the y axis

params.img_name = string(ls(params.img_path));
params.img_name = params.img_name(3:end);
params.final_name = string(ls(params.final_path));
params.final_name = params.final_name(3:end);
params.tideFunction = 'cBathyTide';  % tide level function for evel ��λ
%%%%%
params.fs = 2;                      % ����Ƶ��sampling frequency
params.N = length(params.final_name); % ��Ƶͼ�񳤶�Video image length
params.dxm = 0.5;                   % local����ϵ x�����ֱ���  X axis distance resolution of local coordinate system ��λ ��/���� 
params.dym = 0.5;                   % local����ϵ y�����ֱ���  Y-axis distance resolution of local coordinate system  Unit meter/pixel
params.prm = params.dxm;
params.dist = 1;                    %%% Notice: timeCor cBathy ���ݷֱ���ʵ�ʶ��Դ�Ϊ�� %%%
params.fix_time = 3;                % �̶�ʱ��Ϊ3s
% params.pred_range = [10 40];      % �������Ʒ�Χ
% params.pred_k = 1;


params.xy_range = params.xy_min_max;
params.xy_want = params.xy_min_max;

params.df = params.fs / params.N;          % fft����ÿ����������Ӧ��Ƶ�ʼ��������˵����fs/N;
params.dt = 1 / params.fs;          % delta t�����������
params.t = 0 : params.dt : (params.N - 1) * params.dt; % ����Ҳ�������ã��Զ�����
params.f = 0 : params.df : (1 / (2 * params.dt) - params.df );
% params.tideFunction = 'cBathyTide';  % ��λУ���ĺ���У��

%%%%%%%   �㷨����   %%%%%%%
params.MINDEPTH = 0.25;             % ��ʼ����С�����Ϊ��ʼֵ
params.minValsForBathyEst = 4;      % f_k�Եĸ���

params.QTOL = 0.40;                  % ��϶ȵ������ֵ˵��������ϣ�Ҫreject
params.minLam = 10;                 % min normalized eigenvalue to proceed ���е���С��һ������ֵ
params.Lx = 6*params.dxm;           % �����ѡ��,X���ϵ�
params.Ly = 6*params.dym;           % �����ѡ��,Y���ϵ�
params.kappa0 = 2;                  % �������ӣ��밶ԽԶ�����ҪԽ��
params.DECIMATE = 1;                % decimate pixels to reduce work load. ���������Լ��ٹ������ı�־λ
params.maxNPix = 80;                % max num pixels per tile (decimate excess) ÿ��������������80�����ص�
 
% ѡ��Ҫ������Ƶ�ʷ�Χ
params.fB = [1/20: 1/50: 1/4];		% ö�ٿ��ܵ�Ƶ��ֵ
params.nKeep = 6;                   % ������ҪƵ�ʵĸ���
params.dfB = mean(diff(params.fB)); % dfBs


% �Ƿ���debug����
params.DEBUG = 0;                       % ���������ѡ��
params.debug.production = 0;            % 1������0�ر�
params.debug.DOPLOTSTACKANDPHASEMAPS = 1;  % ����Ƶ�ʶ�Ӧ����λͼ
params.debug.DOSHOWPROGRESS = 0;		  % ����ͼչʾ
params.debug.DOPLOTPHASETILE = 0;		  % ÿ�����ص��eof���
params.debug.TRANSECTX = 0;		  % ��ͼѡ�����x��Ľض��棬���Ϊx����ֵ
params.debug.TRANSECTY = 0;		  % ��ͼѡ�����y��Ľض��棬���Ϊy����ֵ

% default offshore wave angle.  For search seeds.
params.offshoreRadCCWFromx = 0;