function A = diffSafe(X, n, dim)
% diffSafe - applies the diff function to a 1 or 2-D matrix, and cuts off any
% "outliers". 
%  
% A = diffSafe(X, n, dim)
% INPUTS:   X           = matrix
%           n           = order of the diff function applied
%           dim         = matrix dimension in which to apply the diff
%                         function
% OUTPUTS:  A           = matrix containing the bounded diff
%
%
%
% Copyright 2025, Victor Suciu, Nobleo Technology & Technische Universiteit
% Eindhoven, Netherlands

A = diff(X, n, dim);

if dim == 1
    A = transpose(A);
    X = transpose(X);
end

for i = 1:size(A, 1)
    A(i, :) = nanMatrixOutliers(A(i, :), method="movmedian_10");
end

% for i = 1:size(A, 3-dim)
%     if dim == 2
%         A(i, :) = nanMatrixOutliers(A(i, :), method="movmedian_10");
%     else
%         A(:, i) = nanMatrixOutliers(A(:, i), method="movmedian_10");
%     end
% end
% A(abs(A)>5) = nan;

% if(anyEq(A, nan))
%     idx = isnan(A);
%     for i = 1:size(A, 1)
%         ids = find(idx(i,:));
%         for k = 1:length(ids)
%             j = ids(k);
%             if(X(i, j) < X(i, j+1))
%                 A(i, j) = X(i, j+1) - 2*pi - X(i, j);
%             else
%                 A(i, j) = X(i, j+1) + 2*pi - X(i, j);
%             end
%         end
%     end
% end

if dim == 1
    A = transpose(A);
end