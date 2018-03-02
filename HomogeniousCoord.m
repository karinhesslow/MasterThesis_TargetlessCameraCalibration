function [ M ] = HomogeniousCoord( R, t )
% Converts a rotation matrix and a translation to a 4x4 homogenious coord.
%
% Input:
% R                 [3,3,N] Rotation matrix
% t                 [3,N]   Translation vector
%
% Output:
% M                 [4,4,N] Homogenious coord.
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

if numel(R) == 9
    M = [[R, t]; 0, 0, 0, 1];
else
    % Paramerters
    N = numel(t(1,:));
    M = zeros(4,4,N);
    
    for i = 1:numel(N)
        M(:,:,i) = [[R(:,:,i), t(:,i)]; 0, 0, 0, 1];
    end
    
end

end

