function A = rotz(ang)
% generates a 3D rotation matrix for a coordinate
% system change FROM one coordinate system TO another
% rotated from the first by an ang about the Z axis shared
% by the two
A = [cos(ang) sin(ang) 0; ...
    -sin(ang) cos(ang) 0; ...
    0 0 1];
end