function plotGtTimeCBathy(world, gt, timeCor, cBathy, f_size)
    
    set(gcf,'RendererMode','manual','Renderer','painters');
    cmap = colormap( 'jet' );
    colormap( flipud( cmap ) );
    
	clf; %清除图像上的内容clear figure
    subplot(131); % the first image is ground truth.
    pcolor(world.x, world.y, gt);
    %之后要记得反转y坐标
    shading flat
    caxis([0 5]); %设置深度的显示范围
    set(gca, 'ydir', 'nor');
    axis equal;
    axis tight;
    xlabel('x (m)','FontSize',f_size);
    ylabel('y (m)','FontSize',f_size);
%     titstr = datestr( epoch2Matlab(str2num(bathy.epoch)), ...
%     'mmm dd yyyy, HH:MM' );
%     title_name = "testBathy";
    title('无人船测量真实值','FontSize',f_size);
    h=colorbar('peer', gca);
    set(h, 'ydir', 'rev');
    set(get(h,'title'),'string', 'h (m)');
    set(gca,'ydir','reverse');

    

    subplot(132);
    pcolor(world.x, world.y, timeCor);

    shading flat
    caxis([0 5]); %设置深度的显示范围
    set(gca, 'ydir', 'nor');
    axis equal;
    axis tight;
    xlabel('x (m)','FontSize',f_size);
    ylabel('y (m)','FontSize',f_size);
%     titstr = datestr( epoch2Matlab(str2num(bathy.epoch)), ...
%     'mmm dd yyyy, HH:MM' );
%     title_name = "testBathy";
    title('timeCor','FontSize',f_size);
    h=colorbar('peer', gca);
    set(h, 'ydir', 'rev');
    set(get(h,'title'),'string', 'h (m)');
    set(gca,'ydir','reverse');
    
   
    
    subplot(133);
    pcolor(world.x, world.y, cBathy);

    shading flat
    caxis([0 5]); %设置深度的显示范围
    set(gca, 'ydir', 'nor');
    axis equal;
    axis tight;
    xlabel('x (m)','FontSize',f_size);
    ylabel('y (m)','FontSize',f_size);
%     titstr = datestr( epoch2Matlab(str2num(bathy.epoch)), ...
%     'mmm dd yyyy, HH:MM' );
%     title_name = "testBathy";
    title('cBathy','FontSize',f_size);
    h=colorbar('peer', gca);
    set(h, 'ydir', 'rev');
    set(get(h,'title'),'string', 'h (m)');
    set(gca,'ydir','reverse');

    
end

