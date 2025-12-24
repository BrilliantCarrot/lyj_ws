function angle = Angle_trim(angle)
    if angle < 0 
        angle = angle + 2*pi;
    elseif angle > 2*pi
        angle = angle - 2*pi;
    end
end