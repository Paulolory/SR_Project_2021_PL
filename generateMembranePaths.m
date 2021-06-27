function [segmentCellArray, surfCellArray] = generateMembranePaths(numSamples, spaceLimits)
%generateMembranePaths Generate segments that describe the 3D MATLAB Logo
%   This function generates segments for the border and two defining cross
%   sections of the MATLAB logo, as described by the membrane function,
%   using segments with a total of NUMSAMPLES number of samples. The output
%   is scaled to the xyz limits specified by the six element vector
%   SPACELIMITS.

%% Define the scale
xBds = spaceLimits(1,:);
yBds = spaceLimits(2,:);
zBds = spaceLimits(3,:);

%% Define the membrane

n = 51;
p = (n-1)/2;
x = linspace(xBds(1), xBds(2), n);
y = linspace(yBds(1), yBds(2), n);
z = (zBds(2)-zBds(1))*membrane(1,p) + zBds(1);
surfCellArray = {x y z};

%% Use interpolation to sample cross-sections in the XZ and YZ planes

xzSlice = zeros(numSamples, 3);
yzSlice = zeros(numSamples, 3);

% Define X and Y points for cross section in XZ plane at Y = 0
xzSlice(:,1) = linspace(xBds(1), xBds(2), numSamples);
xzSlice(:,2) = .35*(yBds(2) - yBds(1)) + yBds(1);

% Define X and Y points for cross section in YZ plane at X = 0
yzSlice(:,1) = .65*(xBds(2) - xBds(1)) + xBds(1);
yzSlice(:,2) = linspace(yBds(1), yBds(2), numSamples);

xzSlice(:,3) = interp2(x,y,z,xzSlice(:,1),xzSlice(:,2));
yzSlice(:,3) = interp2(x,y,z,yzSlice(:,1),yzSlice(:,2));

%% Do the same to get the boundaries
xzLowerBorder = zeros(numSamples, 3);
xzUpperBorder = zeros(numSamples, 3);
yzLowerBorder = zeros(numSamples, 3);
yzUpperBorder = zeros(numSamples, 3);

% Go sequentially around the border
xzLowerBorder(:,1) = linspace(xBds(1), xBds(2), numSamples);
xzLowerBorder(:,2) = yBds(1);
xzLowerBorder(:,3) = interp2(x,y,z,xzLowerBorder(:,1),xzLowerBorder(:,2));

yzUpperBorder(:,1) = xBds(2);
yzUpperBorder(:,2) = linspace(yBds(1), yBds(2), numSamples);
yzUpperBorder(:,3) = interp2(x,y,z,yzUpperBorder(:,1),yzUpperBorder(:,2));

xzUpperBorder(:,1) = linspace(xBds(2), xBds(1), numSamples);
xzUpperBorder(:,2) = yBds(2);
xzUpperBorder(:,3) = interp2(x,y,z,xzUpperBorder(:,1),xzUpperBorder(:,2));

% YZ Borders
yzLowerBorder(:,1) = xBds(1);
yzLowerBorder(:,2) = linspace(yBds(2), yBds(1), numSamples);
yzLowerBorder(:,3) = interp2(x,y,z,yzLowerBorder(:,1),yzLowerBorder(:,2));

%% Combine all the segments in an output cell array

segmentCellArray = {xzLowerBorder, yzUpperBorder, xzUpperBorder, yzLowerBorder, ...
    xzSlice, yzSlice};

end
    