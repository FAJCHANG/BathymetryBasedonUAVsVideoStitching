function [graph, goodAone] = OneGraph( trackA, alpha)
    nWindow = trackA.nWindow;
    nLabel = trackA.nLabel;
    videoH = trackA.videoHeight;
    videoW = trackA.videoWidth;
    graph = Graph(nWindow, nLabel, 1);
    wSize = trackA.wSize;
    reGraph = 1;
    if reGraph == 0
        load('graphA.mat') ;
    else
        for windowIndex = 1:nWindow
            fprintf('\nwindow: %d\n', windowIndex);
            left = (windowIndex - 1) * wSize / 2 + 1;
            right= (windowIndex + 1) * wSize / 2; 
            %求取最大分类label
            nU_current = max( max( max(  trackA.labels(:, left:left + wSize / 2 - 1, 2)  )  ), max( max(trackA.labels(:, left + wSize / 2:right, 1))) );
           
            if windowIndex < nWindow
                %求取最大分类label
                nU_next = max(max(max(trackA.labels(:, left + wSize / 2:left + wSize - 1, 2))), max(max(trackA.labels(:, left + wSize:right + wSize / 2, 1))));
                left = windowIndex * wSize / 2 + 1;
                right= (windowIndex + 1) * wSize / 2; 
                
                track = trackA;
                t_nLabel1 = nU_current;
                t_nLabel2 = nU_next;
                
                for u = 1:t_nLabel1 % 这里应该是计算权重，具体请查看论文
                    for v = 1:t_nLabel2                        
                        % u in window windowIndex, v in window (windowIndex + 1) 
                        common = track.labels(:, left, 1) == u & track.labels(:, left, 2) == v;
                        common_ = track.labels(:, right, 1) == u & track.labels(:, right, 2) == v;
                        if common ~= common_
                            error('?'); % these two should be the same
                        end 
                        nCommon = sum(common);

                        if nCommon < 4
                            edgeWeight = 1e8;
                            graph.setEdgeWeight(nodeIndex(1, windowIndex, u, nLabel, nWindow), ...
                                nodeIndex(1, windowIndex + 1, v, nLabel, nWindow), edgeWeight);
                            continue;
                        end

                        % compute the trajectory matrix
                        W2 = track.points(common, left:right, :);                    
                        a1 = W2(:, :, 1); a2 = W2(:, :, 2);
                        a1 = a1 ./ videoW; a2 = a2 ./ videoH;
                        a1a2 = [reshape(a1', [nCommon * wSize / 2 1]) reshape(a2', [nCommon * wSize / 2 1])];
                        W = reshape(a1a2', [wSize nCommon]);

                        fprintf('%d -> %d nCommon = %d, Rank = %d\n', u, v, nCommon, rank(W, 0.00));

%                             function index = nodeIndex(v, w, u, nL, nW)
%                              节点的索引值
%                                 index = (v-1)*nL*nW + (w-1)*nL + u;
%                             end

                        % compute the edge weight 
                        edgeWeight = exp( - alpha * nCommon) * rank(W, 0.00)/4;
                        graph.setEdgeWeight(nodeIndex(1, windowIndex, u, nLabel, nWindow), ...
                            nodeIndex(1, windowIndex + 1, v, nLabel, nWindow), edgeWeight);                        
                    end
                end%for u = 1:t_nLabel1
            end%if
        end%for
        save('graphA.mat', 'graph');
    end%if
    
    graph.computeShortestPathForVideo1From0tok();
    graph.computeShortestPathForVideo1Fromkto0();
    graph.getShortestPathVideo1();
    graph.getShortestPathVideo1fromkto0();
    
    pathA = graph.getShortestPathVideo1();
    
    goodAone = zeros(nWindow, nLabel);
    for windowIndex = 1:nWindow
        fprintf('Window %2d\n', windowIndex);
        goodAone(windowIndex, pathA.nodeRelative(windowIndex) + 1) = 1;
        fprintf('%d\t%f*\n', pathA.nodeRelative(windowIndex) + 1, pathA.dis);
        for testLabel = 1:nLabel
            node = nodeIndex(1, windowIndex, testLabel, nLabel, nWindow);
            path = graph.getShortestPathAcrossSpecificNode(node);
            dis = path.dis;
            fprintf('%d\t%f\t', testLabel, dis);
            if dis < max(pathA.dis, 2)
                goodAone(windowIndex, testLabel) = 1;
                fprintf('[');
                [left, right] = getNearest(graph, 1, windowIndex, testLabel, nLabel, nWindow);
                if windowIndex > 1 && left>0
                    goodAone(windowIndex - 1, left) = 1;
                    fprintf('%2d', left);
                end
                if windowIndex < nWindow && right > 0
                    goodAone(windowIndex + 1, right) = 1; 
                    fprintf('%2d', right);
                end 
                fprintf(']');
            end            
        end
        fprintf('\n');
    end
    
end

function index = nodeIndex(v, w, u, nL, nW)
    index = (v-1)*nL*nW + (w-1)*nL + u;
end


function [left, right] = getNearest(graph, videoIndex, windowIndex, labelIndex, nLabel, nWindow)    
    node = nodeIndex(videoIndex, windowIndex, labelIndex, nLabel, nWindow);
    left = 0;
    right = 0;
    if windowIndex > 1
        [edge, nearest] = min(squeeze(graph.adjacentMatrix(node, node - labelIndex - nLabel + 1:node - labelIndex)));
        if edge < 1e8
            left = nearest;
        end
    end
    if windowIndex < nWindow
        [edge, nearest] = min(squeeze(graph.adjacentMatrix(node, node - labelIndex + nLabel + 1:node - labelIndex + nLabel + nLabel)));
        if edge < 1e8
            right = nearest;
        end
    end
end