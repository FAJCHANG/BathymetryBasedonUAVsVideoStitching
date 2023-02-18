% ģ��cBathy������

run('../bathyParams');
addpath('./algoV2/makeTimeStack');

save_info.path = params.data_save_path;
save_info.name = 'data_struct';


plotFlag.plot_raw_filter_f_distribution = 0; 



time_info = [2022 05 18 17 02 0];
%%
% ���xyz��data�ṹ��
[xyz, data, t, N] = makeTimeStack(params.final_path, save_info, time_info, params);
% params.N = N;

%%
data_lowpass = lowPassFliter(data, params);

if plotFlag.plot_raw_filter_f_distribution
    figure(99)
    plot(1 : params.N , abs(fft(data_lowpass)),'r');
    hold on;
    plot(1 : params.N, abs(fft(detrend(data))), 'b');
    pause;
    close(99)
end
disp('ok');
%% ����һ��ʱ����ص�Ƶ��
% dbstop if all error  % �������
G = fft(detrend(data));
x = xyz(:, 1);
y = xyz(:, 2);
z = xyz(:, 3);
data_id = 1;

for y_pos = min(y) : params.dist : max(y)
    longshore_id = find(y == y_pos);
    sub_x = x(longshore_id, :);
    sub_y = y(longshore_id, :);
    sub_G = G(: , longshore_id);    % x���ж�ӦG���У���һһ��Ӧ��

    for x_id = 1 : length(sub_x)
        range = find(sub_x <= sub_x(x_id) + 20 ...
                    & sub_x >= sub_x(x_id) - 20);
        range_G = sub_G(:, range);
%         ff(:, data_id) = findMostCorFreq(range_G, params)'; % ��ȡnKeep����Ƶ��
        mf = findMostCorFreq(range_G, params)'; % ��ȡnKeep����Ƶ��
        for i = 1 : length(mf)
           [~, ff_id(:, i)] = min(abs(params.f - mf(i)));
        end
        keep_ff = params.f(ff_id');
        ff = [];
        for i = 2 : length(params.f) % ֱ������û��Ҫȥȥ��
            if ~ismember(params.f(i), keep_ff)
                ff = [ff params.f(i)];
            end
        end
        data_final(:, data_id) = fftFilter(data_lowpass(:, data_id), ff, params);
        data_id = data_id + 1;
    end
    disp(['progress:' num2str((y_pos - min(y)) / (max(y) - min(y)) * 100) '% completed']);
end

%%
    save([save_info.path 'data_final'], 'data_final');
    disp('mainMakeTimeStack finished');
%%
% figure(23)
% plot(data_final(:, 30050),'r');
% hold on;
% plot(data_final(:, 29045), 'g');
% xlabel('ʱ��(��)');
% ylabel('�ź�ǿ��');
% legend('150m','145m', 'fontsize', 15);
% axis([0 100 -3 3]);