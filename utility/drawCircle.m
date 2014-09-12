function h = drawCircle( radius, center, varargin )
%CIRCLE Draws a circle on the current axes
%   Detailed explanation goes here
if nargin == 3
    facecolor = varargin{1};
else
    facecolor = 'r';
end
h = rectangle('Position',[center(1)-radius, center(2)-...
    radius,2*radius, 2*radius],'Curvature', [1,1],'FaceColor',facecolor);

end

