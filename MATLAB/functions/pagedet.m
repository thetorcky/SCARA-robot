function determinant = pagedet(A)
% pagedet - Calculates the page-wise determinant of a 4-D array
%
% determinant = pagedet(A)
% INPUTS:   A            = 4-D array
%
% OUTPUTS:  determinant  = matrix containing the determinant of page
%                          (:,:,i,j) in element (i,j)
%
%
%
% Copyright 2025, Victor Suciu, Nobleo Technology & Technische Universiteit
% Eindhoven, Netherlands


eigsA = pageeig(A);
determinant(:,:) = prod(eigsA(:,1,:,:));
