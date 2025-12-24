
function Result = interp_1(XI,YI,X)
   Index =  Find_Index(XI,length(XI),X);
   Result = (YI(Index+1)-YI(Index))/(XI(Index+1)-XI(Index)) .* (X-XI(Index)) + YI(Index);
end