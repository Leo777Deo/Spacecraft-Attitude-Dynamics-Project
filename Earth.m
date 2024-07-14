function Earth
%
% Earth: The function gives the Planet's texture loaded in a plot.
%
% No inputs are needed.
%
% An image of the Earth must be associated in the folder.
%
%-----------------------------------------------------------------------

% Data definition:
hold on;
radiusEarth = 6378.15; %km
[xs,ys,zs] = sphere;
xs = radiusEarth*xs;
ys = radiusEarth*ys;
zs = radiusEarth*zs; 
s = surf(xs,ys,-zs);
set(s,'edgecolor','none')
sph1 = findobj('Type','surface');

%Plot Earth on the sphere: 
im1 = imread('earth.png');
set(sph1,'CData',im1,'FaceColor','texturemap','FaceAlpha',1);

grid on;
xlabel('x');
ylabel('y');
zlabel('z');

axis equal;

end