function I = DrawFeature( I , f, color, shape)
    I = insertMarker(I, f, shape, 'color', color, 'size', 6);        
    I = insertMarker(I, f, shape, 'color', color, 'size', 5);
    I = insertMarker(I, f, '*', 'color', color, 'size', 6);        
    I = insertMarker(I, f, '+', 'color', color, 'size', 6);        
    I = insertMarker(I, f, shape, 'color', color, 'size', 7);    
%     I = insertMarker(I, f, shape, 'color', color, 'size', 8);   
    I = insertMarker(I, f, shape, 'color', 'black', 'size', 6);
end

