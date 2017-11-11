function data = normalize(data)
% function data = normalize(data)
% data: MxN. M: the number of data, N: size of one data

mag = sqrt(sum(data.^2,2));
data = bsxfun(@rdivide,data,mag);

end
