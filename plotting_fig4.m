figure (1)
hold on

% intermediate value 1, fcn #20

optimalWayPoints=[1389.389714	1391.869361	1273.257297	1202.880962	1132.444654	1034.089032	1043.493236	1104.331491	1285.638734	1324.938866]';

optimalWayPoints = [0 sizeY/2; reshape([xWayPoints(2:end-1)'; optimalWayPoints'],2,[])'; sizeX sizeY/2];

xWayPoints = optimalWayPoints(:,1);
yWayPoints = optimalWayPoints(:,2);
plot(xWayPoints,yWayPoints,"Color","#7E2F8E","LineStyle","none","Marker",".","MarkerSize",16);
pathPoints = waypointsToPath([xWayPoints,yWayPoints],method,sizeX,sizeY,integrationFineness);
plot( pathPoints(:,1),pathPoints(:,2),"Color","#7E2F8E","LineWidth",2);

it1=getTimeFromPath(pathPoints, dHx, dHy,V0, numWayPoints, Heights);

% intermediate value 2 iteration #50

optimalWayPoints=[1357.613204	1338.73671	1228.295806	1097.78668	1017.522593	943.8975043	966.2884421	1041.81655	1100.031998	1219.266049]';

optimalWayPoints = [0 sizeY/2; reshape([xWayPoints(2:end-1)'; optimalWayPoints'],2,[])'; sizeX sizeY/2];

xWayPoints = optimalWayPoints(:,1);
yWayPoints = optimalWayPoints(:,2);
plot(xWayPoints,yWayPoints,"Color","#7E2F8E","LineStyle","none","Marker",".","MarkerSize",16);
pathPoints = waypointsToPath([xWayPoints,yWayPoints],method,sizeX,sizeY,integrationFineness);
plot( pathPoints(:,1),pathPoints(:,2),"Color","#7E2F8E","LineWidth",2);

it2=getTimeFromPath(pathPoints, dHx, dHy,V0, numWayPoints, Heights);

% intermediate value 3 iteration #80

optimalWayPoints=[1346.676474	1298.001324	1154.979092	1042.725236	992.9307617	944.9544137	932.1444646	969.4034701	1064.008193	1167.157098]';

optimalWayPoints = [0 sizeY/2; reshape([xWayPoints(2:end-1)'; optimalWayPoints'],2,[])'; sizeX sizeY/2];

xWayPoints = optimalWayPoints(:,1);
yWayPoints = optimalWayPoints(:,2);
plot(xWayPoints,yWayPoints,"Color","#7E2F8E","LineStyle","none","Marker",".","MarkerSize",16);
pathPoints = waypointsToPath([xWayPoints,yWayPoints],method,sizeX,sizeY,integrationFineness);
plot( pathPoints(:,1),pathPoints(:,2),"Color","#7E2F8E","LineWidth",2);

it3=getTimeFromPath(pathPoints, dHx, dHy,V0, numWayPoints, Heights);

% intermediate value 4, iteration #140

optimalWayPoints=[1326.186126	1242.282434	1085.638074	983.8822497	946.3703128	923.7463155	924.453204	964.8672064	1060.753568	1174.197586]';

optimalWayPoints = [0 sizeY/2; reshape([xWayPoints(2:end-1)'; optimalWayPoints'],2,[])'; sizeX sizeY/2];

xWayPoints = optimalWayPoints(:,1);
yWayPoints = optimalWayPoints(:,2);
plot(xWayPoints,yWayPoints,"Color","#7E2F8E","LineStyle","none","Marker",".","MarkerSize",16);
pathPoints = waypointsToPath([xWayPoints,yWayPoints],method,sizeX,sizeY,integrationFineness);
plot( pathPoints(:,1),pathPoints(:,2),"Color","#7E2F8E","LineWidth",2);

it4=getTimeFromPath(pathPoints, dHx, dHy,V0, numWayPoints, Heights);

% final value 5
optimalWayPoints = [1325.395852	1241.021181	1084.530037	983.5758867	947.5748332	927.3240601	928.0299757	969.86088	1070.633763	1179.549993]';

optimalWayPoints = [0 sizeY/2; reshape([xWayPoints(2:end-1)'; optimalWayPoints'],2,[])'; sizeX sizeY/2];

xWayPoints = optimalWayPoints(:,1);
yWayPoints = optimalWayPoints(:,2);
plot(xWayPoints,yWayPoints,"Color","#7E2F8E","LineStyle","none","Marker",".","MarkerSize",16);
pathPoints = waypointsToPath([xWayPoints,yWayPoints],method,sizeX,sizeY,integrationFineness);
plot( pathPoints(:,1),pathPoints(:,2),"Color","#7E2F8E","LineWidth",2);

it5=getTimeFromPath(pathPoints, dHx, dHy,V0, numWayPoints, Heights);

%% FCN - CALCULATE TRAVEL TIME
function [travelTime, dP_3d] = getTimeFromPath(pathPoints, dHx, dHy,V0,numWayPoints, Heights, sizeX,sizeY,method,integrationFineness)

% Set physical parameters
m = 170; % mass, [kg]
g_mars = 3.71; % mars gravity, [m/s^2]
u_rr = 0.6; % rolling resistance [-]
density = 0.0145; % air density on mars [kg/m^3]
wheel_r = 0.5; % wheel radius, [m]
Cd = 0.33; % Drag coeff [-]
Ax = (2*1.5); % X-sect area of rover, [m^2]
T_in = u_rr*m*g_mars/wheel_r; % Input motor torque, [N-m], set to approximately match rolling resistance

if isvector(pathPoints)
    xWayPoints = linspace(0,sizeX,numWayPoints+2)';
    pathPoints = [0 sizeY/2; reshape([xWayPoints(2:end-1)'; pathPoints'],2,[])'; sizeX sizeY/2];
    pathPoints = waypointsToPath(pathPoints,method,sizeX,sizeY,integrationFineness);
end

dP = diff(pathPoints);

% Interpolate the vector field at all the points in pathPoints.
Grad_interp = [interp2(dHx,pathPoints(1:end-1,1)+1,pathPoints(1:end-1,2)+1,"linear") ...
    interp2(dHy,pathPoints(1:end-1,1)+1,pathPoints(1:end-1,2)+1,"linear")];
Height_interp = [interp2(Heights,pathPoints(1:end,1)+1,pathPoints(1:end,2)+1,"linear")];

% get 3D heights
diff_height = diff(Height_interp);
dP_3d = [dP, diff_height];

grav_effects = (sum((-g_mars*m).*Grad_interp.*dP,2))./sqrt(sum(dP.^2,2)); %Z*g vector along path, normalized
drag = (1/2)*density*Cd*V0^2; %dependent on velocity, but small
F_in = T_in*wheel_r;
F_rr = g_mars*m*u_rr; %constant scalar

V_add = (1/(m*V0))*sqrt(sum(dP_3d.^2,2)).*( grav_effects - drag + 4*F_in - 4*F_rr);
dx = sqrt(sum(dP_3d.^2,2)); %dx is the 3D length of each subinterval
dt = dx./(V0+V_add);  %dT = dP/dV
travelTime = sum(dt);

end


%% FCN - MAKE THE CONTOUR MAP
function map = Generate_contour(SZX,SZY)

Fineness = 0.1;
if ~exist("SZX","var")
    SZX = 50;
    SZY = 50;
end

N = 50; % Various parameters used in generating a random "smooth" matrix
NL = 40;
NP = 500;
rx = randn(NL,N);
rx = interpft(rx,NP);
ry = randn(NL,N);
ry = interpft(ry,NP);
I = (rx*ry');

[xgi,ygi] = meshgrid(linspace(1,2 + 498*Fineness,SZX+1),linspace(1,2 + 498*Fineness,SZY+1));
map = 1*interp2(1:500,1:500,I,xgi,ygi);
end


%% FCN - MAKE THE PATH
function pathPoints = waypointsToPath(p,method,sizeX,sizeY,fineness)
% Interpolate the curve based on the discrete waypoints to generate a
% continuous path.

nP = size(p,1);
pathPoints = [interp1(1:nP,p(:,1),linspace(1,nP,fineness)',method,"extrap") ...
    interp1(1:nP,p(:,2),linspace(1,nP,fineness)',method,"extrap")];

end