function helperPlotScenarioSurface( srf,localOrigin,useECEF,simTime )
% This helper function plots a visualization of scenario surface objects.
% If the surface is homogeneous, only the boundary will be plotted.

if nargin < 2 || isempty(localOrigin)
    localOrigin = srf.LocalOrigin;
end

if nargin < 3 || isempty(useECEF)
    useECEF = false;
end

if nargin < 4
    simTime = 0;
end

if isa(srf,'radarfusion.internal.scenario.StaticSurfaceModel')
    H = srf.getHeightMap;
else
    H = srf.getHeightMap(simTime);
end

if isempty(H)
    % Plot boundary. If the boundary is unbounded in any direction, arrows
    % will be plotted when possible to indicate the direction in which the
    % surface is unbounded.

    b = srf.Boundary;

    if srf.IsEarthCentered && useECEF
        b(3,:) = 0;
        b = fusion.internal.frames.lla2ecef(b.').';
    end
    
    spec = {'color','black','linewidth',3};

    plot(nan,nan)
    hold on
    
    edgeType = zeros(2,2);
    
    srf = diff(b,1,2);
    srf = srf(~isinf(srf));
    if ~isempty(srf)
        srf = max(srf);
    else
        srf = 1;
    end
    
    % get X edge types
    if ~isinf(b(1,1))
        if isinf(b(1,2))
            edgeType(1,1) = 1;
        else
            edgeType(1,1) = 2;
        end
    elseif ~isinf(b(1,2))
        edgeType(1,2) = 1;
    end
    
    % get Y edge types
    if ~isinf(b(2,1))
        if isinf(b(2,2))
            edgeType(2,1) = 1;
        else
            edgeType(2,1) = 2;
        end
    elseif ~isinf(b(2,2))
        edgeType(2,2) = 1;
    end
    
    % draw X edge at lower Y
    if ~isinf(b(2,1))
        if edgeType(1,1) == 1
            quiver(b(1,1),b(2,1),srf,0,'autoscale','off',spec{:});
        elseif edgeType(1,1) == 2
            line([b(1,1) b(1,2)],b(2,1)*[1 1],spec{:})
        end
        if edgeType(1,2) == 1
            quiver(b(1,2),b(2,1),-srf,0,'autoscale','off',spec{:});
        end
    end
    
    % draw X edge at upper Y
    if ~isinf(b(2,2))
        if edgeType(1,1) == 1
            quiver(b(1,1),b(2,2),srf,0,'autoscale','off',spec{:});
        elseif edgeType(1,1) == 2
            line([b(1,1) b(1,2)],b(2,2)*[1 1],spec{:})
        end
        if edgeType(1,2) == 1
            quiver(b(1,2),b(2,2),-srf,0,'autoscale','off',spec{:});
        end
    end
    
    %  draw Y edge at lower X
    if ~isinf(b(1,1))
        if edgeType(2,1) == 1
            quiver(b(1,1),b(2,1),0,srf,'autoscale','off',spec{:});
        elseif edgeType(2,1) == 2
            line(b(1,1)*[1 1],[b(2,1) b(2,2)],spec{:})
        end
        if edgeType(2,2) == 1
            quiver(b(1,1),b(2,2),0,-srf,'autoscale','off',spec{:});
        end
    end
    
    %  draw Y edge at upper X
    if ~isinf(b(1,2))
        if edgeType(2,1) == 1
            quiver(b(1,2),b(2,1),0,srf,'autoscale','off',spec{:});
        elseif edgeType(2,1) == 2
            line(b(1,2)*[1 1],[b(2,1) b(2,2)],spec{:})
        end
        if edgeType(2,2) == 1
            quiver(b(1,2),b(2,2),0,-srf,'autoscale','off',spec{:});
        end
    end
    
else
    % Plot surface height data

    [x,y] = meshgrid(H.X,H.Y);
    z = H.Z;
    if srf.IsEarthCentered
        if useECEF
            pts = fusion.internal.frames.lla2ecef([x(:) y(:) z(:)]);
        else
            pts = fusion.internal.frames.lla2enu([x(:) y(:) z(:)],localOrigin(:).');
        end
        x = reshape(pts(:,1),size(z));
        y = reshape(pts(:,2),size(z));
        z = reshape(pts(:,3),size(z));
    end
    
    % Decrease alpha with map size
    n = max(size(z));
    alpha = 1/(1+n/100*log2(n));

    surf(x,y,z,'EdgeAlpha',alpha);
    axis equal
    
    cmap = getSurfaceColormap(srf);
    colormap(gca,cmap);
    
end

end

function cmap = getSurfaceColormap( srf )

if isa(srf,'radar.scenario.LandSurface') || isa(srf,'fusion.scenario.GroundSurface')
    cmap0 = hsv2rgb([5/12 1 0.4; 0.25 0.2 1; 5/72 1 0.4]);
else
    cmap0 = hsv2rgb([2/3 1 0.2; 2/3 1 1; 0.5 1 1]);
end

idx1 = 1:3;
idx2 = linspace(1,3,128);
cmap(:,1) = interp1(idx1,cmap0(:,1),idx2);
cmap(:,2) = interp1(idx1,cmap0(:,2),idx2);
cmap(:,3) = interp1(idx1,cmap0(:,3),idx2);

end