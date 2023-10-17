% Define the properties of the ULA
numElements = 10; % Number of elements in the array
elementSpacing = 0.5; % Spacing between elements in wavelengths
ula = phased.ULA('NumElements',numElements,'ElementSpacing',elementSpacing);

% Define the source direction
az = 20; % Azimuth angle in degrees
el = 30; % Elevation angle in degrees
src = phased.Freespace('PropagationSpeed',physconst('LightSpeed'),'OperatingFrequency',1e9,...
    'Direction',[az;el],'TwoWayPropagation',false);

% Generate the received signal at the array
x = src(ula());

% Estimate the DOA using the MUSIC algorithm
doa = phased.MUSICEstimator('SensorArray',ula,'OperatingFrequency',1e9,'NumSignals',1);
[~,ang] = doa(x);

% Calculate the array response pattern over a range of angles
azRange = -90:0.5:90; % Range of azimuth angles to evaluate
elRange = -90:0.5:90; % Range of elevation angles to evaluate
[azGrid,elGrid] = meshgrid(azRange,elRange);
response = pattern(ula,1e9,[azGrid(:)';elGrid(:)'],0,'PropagationSpeed',physconst('LightSpeed'));

% Reshape the response into a 2D grid
response = reshape(response,length(azRange),length(elRange));

% Plot the 3D graph of azimuth, elevation, and power
figure;
surf(azRange,elRange,abs(response));
xlabel('Azimuth Angle (degrees)');
ylabel('Elevation Angle (degrees)');
zlabel('Power');
title('Array Response Pattern');
