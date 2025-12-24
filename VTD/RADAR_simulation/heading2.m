function chi = heading2(P1, P2)
    dx  = P2(1) - P1(1);
    dy  = P2(2) - P1(2);
    chi = atan2(dy, dx);      % [-π, π]
end