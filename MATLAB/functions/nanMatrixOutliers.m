function B = nanMatrixOutliers(A, options)
% nanMatrixOutliers - Replaces outliers present in a matrix with NaN.
%  
% B = nanMatrixOutliers(A)
% INPUTS:   A           = a matrix
%           options     = name-value pairs cointaining plotting preferences
%           - method             = specifies the outlier detection method,
%                                  to be passed on to isoutlier
%           - nMAD               = the number of MADs away from the median
%                                  that a value has to be for it to be
%                                  considered an outlier. Implies the
%                                  "median" detection method.
%
% OUTPUTS:  B           = matrix A with all outliers replaced with NaN
%
%
%
% Copyright 2025, Victor Suciu, Nobleo Technology & Technische Universiteit
% Eindhoven, Netherlands

arguments
    A double
    options.method string
    options.nMAD double = 3
end

B = A;

if isfield(options, "method")
    if options.method == "movmedian_10"
        mask = isoutlier(B, "movmedian", 10);
    end
else
    if ndims(B) > 1 && size(B, 1) > 1 && size(B, 2) > 1
        c=-1/(sqrt(2)*erfcinv(3/2));
        medA = median(A, 'all', "omitmissing");
        MAD = c*median(abs(A-medA), 'all', "omitmissing");
        
        B(B < medA - options.nMAD * MAD) = nan;
        B(B > medA + options.nMAD * MAD) = nan;
        return
    else
        mask = isoutlier(B);
    end
end

B(mask) = nan;