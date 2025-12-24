function [result] = Find_Index(Ref_Data, Loop_End, Data)
    result = Loop_End - sum(Data<Ref_Data);
    if result >0 && Loop_End < 0
        return;
        elseif(result == 0)
            result = 1;
        elseif(result == Loop_End)
            result = Loop_End-1;
    end
end