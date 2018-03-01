function R = rotmat(p, a)

switch a
    case 'x'
        R = [1, 0, 0;
              0, cos(p), -sin(p);
              0, sin(p), cos(p)];
    case 'y'
        R = [cos(p), 0, sin(p);
              0, 1, 0;
              -sin(p), 0, cos(p)];
    case 'z'
        R = [cos(p), -sin(p), 0;
              sin(p), cos(p), 0;
              0, 0, 1]; 
    otherwise
        R = eye(3);
end

end

rewrw
