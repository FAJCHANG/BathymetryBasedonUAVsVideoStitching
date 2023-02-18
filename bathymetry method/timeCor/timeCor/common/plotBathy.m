% 对水深画图
%
function plotBathy(WorldCor, DepthInfo, title_name)
    % set up the figure
    set(gcf,'RendererMode','manual','Renderer','painters');
    cmap = colormap( 'jet' );
    colormap( flipud( cmap ) );

    clf;
%     subplot(121);
%     WorldCor.y = flipud(WorldCor.y);
    pcolor(WorldCor.x, WorldCor.y, DepthInfo);
    %之后要记得反转y坐标
    shading flat
    caxis([0 6]); %设置深度的显示范围
    set(gca, 'ydir', 'nor');
    axis equal;
    axis tight;
    xlabel('long shore(m)');
    ylabel('cross shore(m)');
%     titstr = datestr( epoch2Matlab(str2num(bathy.epoch)), ...
%     'mmm dd yyyy, HH:MM' );
%     title_name = "testBathy";
    title(title_name);
    h=colorbar('peer', gca);
    set(h, 'ydir', 'rev');
    set(get(h,'title'),'string', 'h (m)');
    
%     set(gca,'ydir','reverse','xaxislocation','top');
     set(gca,'ydir','reverse');
    return;
end

