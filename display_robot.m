%
% Author Patric Jensfelt
%
function [h] = display_robot(x,c)

if nargin < 4
    c = 'r';
end

if nargin < 5
    drawDirectionVector = 0;
end

global Length Width

X = [-0.5*Length 0.5*Length 0.5*Length -0.5*Length -0.5*Length;
    0 0 0.25*Length 0.25*Length 0];

h = plot(x+X(1,:),X(2,:),c,'LineWidth',2);