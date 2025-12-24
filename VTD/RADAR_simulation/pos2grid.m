function [ix, iy] = pos2grid(x,y,X,Y)
    
    
    [idx1, idx2] = find(X(1,:)-x < 50);
    [idx3, idx4] = find(Y(:,1)-y < 50);
    ix = idx3(end);
    iy = idx2(end);

end