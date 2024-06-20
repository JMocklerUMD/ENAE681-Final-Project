%% PARAMETERS
clear all; close all;
V0 = 10;
sizeX = 5000;
sizeY = 2500;
numWayPoints = 10;  
randomSeed = 29; rng(randomSeed); %set the random num so runs can be compared
method ="linear" ; % Interpolation method
integrationFineness = 1000;

%% GENERATE A HEIGHT MAP

figure (1)
hold on

Heights = 25*Generate_contour(sizeX,sizeY); %Create the contour map
[dHx, dHy] = gradient(Heights); %Get gradient information
[Xgrid,Ygrid] = meshgrid(0:sizeX,0:sizeY);

avg_h = mean(abs(Heights), 'all');
fprintf('Average contour height: %f \n', avg_h)

% Create plots!
% quiver(Xgrid,Ygrid, dHx, dHy,"k",'LineWidth',1);
contour(Xgrid,Ygrid, Heights, 15,'LineWidth',1.5)
xlabel('Longitude [m]'); ylabel('Latitude [m]');
xlim([0 sizeX]); ylim([0 sizeY]);
title('Simulated Planet Contour')
colorbar('eastoutside')
grid minor


%% STRAIGHT LINE TIME
% set points for a straight path
xWayPoints = linspace(0,sizeX,numWayPoints+2)';
yWayPoints = sizeY/2 * ones(numWayPoints+2,1);

% create a straight line between points for plotting
pathPoints = waypointsToPath([xWayPoints,yWayPoints],method,sizeX,sizeY,integrationFineness);

% plot over figure 1
plot(pathPoints(:,1),pathPoints(:,2),"k","LineWidth",2);
plot(xWayPoints,yWayPoints,"Color","k","LineStyle","none","Marker",".","MarkerSize",16);

% calculate the time to complete the straight path
[straightLineTime, points] = getTimeFromPath(pathPoints, dHx, dHy,V0,numWayPoints, Heights);
fprintf("Straight Line Travel Time: %f seconds\n",straightLineTime);

[xsol,fval,history,searchdir] = runfmincon;

%% OPTIMIZATION OUTPUTS


function [xsol,fval,history,searchdir] = runfmincon

V0 = 10;
sizeX = 5000;
sizeY = 2500;
numWayPoints = 10;
randomSeed = 29; rng(randomSeed);
method ="linear" ; % Interpolation method
integrationFineness = 1000;
Heights = 25*Generate_contour(sizeX,sizeY); %Create the contour map
[dHx, dHy] = gradient(Heights); %Get gradient information
[Xgrid,Ygrid] = meshgrid(0:sizeX,0:sizeY);

history.x = [];
history.fval = [];
searchdir = [];

% Set the objective fcn
objectiveFun = @(P) getTimeFromPath(P, dHx, dHy,V0,numWayPoints, Heights,...
    sizeX,sizeY,method,integrationFineness);

xWayPoints = linspace(0,sizeX,numWayPoints+2)';
yWayPoints = sizeY/2 * ones(numWayPoints+2,1);
ipt = yWayPoints(2:end-1)'; ipt = ipt(:);
lb = zeros(size(ipt(:)))+2; ub = sizeY.*ones(size(ipt(:)))-2;
lb = zeros(size(ipt(:))); ub = sizeY.*ones(size(ipt(:)));

% Call optimization

options = optimoptions(@fmincon,'OutputFcn',@outfun,... 
    'Display','iter','Algorithm','active-set');
options.MaxFunctionEvaluations = numWayPoints*500;

[xsol,fval] = fmincon(objectiveFun,ipt,[],[],[],[],lb,ub,[],options);
 
 function stop = outfun(x,optimValues,state)
     stop = false;
 
     switch state
         case 'init'
             hold on
         case 'iter'
         % Concatenate current point and objective function
         % value with history. x must be a row vector.
           history.fval = [history.fval; optimValues.fval];
           history.x = [history.x; x'];
         case 'done'
             hold off
         otherwise
     end
 end


end

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
