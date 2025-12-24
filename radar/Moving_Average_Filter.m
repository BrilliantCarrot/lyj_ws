function filteredData = Moving_Average_Filter(data,degree)
windowSize = ((length(data)-1)/360)*degree;
b = ones(1, windowSize) / windowSize;
filteredData = filter(b, 1, data);
end