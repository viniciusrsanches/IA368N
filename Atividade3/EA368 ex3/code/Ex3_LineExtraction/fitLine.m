%---------------------------------------------------------------------
% This function computes the parameters (r, alpha) of a line passing
% through input points that minimize the total-least-square error.
%
% Input:   XY - [2,N] : Input points
%
% Output:  alpha, r: paramters of the fitted line

function [alpha, r] = fitLine(XY)
% Compute the centroid of the point set (xmw, ymw) considering that
% the centroid of a finite set of points can be computed as
% the arithmetic mean of each coordinate of the points.

% XY(1,:) contains x position of the points
% XY(2,:) contains y position of the points

N = max(size(XY));

x_aux = 0;
for i=1:N
  x_aux += XY(1,i);
end
xc = x_aux / N;

y_aux = 0;
for i=1:N
  y_aux += XY(2,i);
end    
yc = y_aux/N;
  

    % compute parameter alpha (see exercise pages)
    sum_nom = 0;
    for i=1:N
      sum_nom += ((XY(1,i)-xc)*(XY(2,i) - yc));
    end
    nom   = -2* sum_nom;
    sum_denom = 0;
    for i=1:N
      sum_denom += ((XY(2,i)-yc)^2 - (XY(1,i)-xc)^2);
    end
    denom = sum_denom;
    alpha = atan2(nom,denom)/2;

    % compute parameter r (see exercise pages)
    r = xc*cos(alpha)+yc*sin(alpha);


% Eliminate negative radii
if r < 0,
    alpha = alpha + pi;
    if alpha > pi, alpha = alpha - 2 * pi; end
    r = -r;
end

end
