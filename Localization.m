propSpeed = 1520;
channelDepth = 200;
OperatingFrequency = 37.5e3; 

isopaths = phased.IsoSpeedUnderwaterPaths('ChannelDepth',channelDepth,...
  'NumPathsSource','Property','NumPaths',10,'PropagationSpeed',propSpeed);

channel1 = phased.MultipathChannel('OperatingFrequency',OperatingFrequency);
channel2 = phased.MultipathChannel('OperatingFrequency',OperatingFrequency);

prf = 1;
pulseWidth = 10e-4;
pulseBandwidth = 1/pulseWidth;
fs = 2*pulseBandwidth;
wav = phased.RectangularWaveform('PRF',prf,'PulseWidth',pulseWidth,...
  'SampleRate',fs);
channel1.SampleRate = fs;
channel2.SampleRate = fs;


figure(1)
rectwav = step(wav);
nsamp = size(rectwav,1);
t = [0:(nsamp-1)]/fs;
plot(t*1000,real(rectwav))
xlabel('Time (millisec)')
ylabel('Amplitude')
title("Signal transmitted by the target") 


projector = phased.IsotropicProjector('VoltageResponse',120);

projRadiator = phased.Radiator('Sensor',projector,...
  'PropagationSpeed',propSpeed,'OperatingFrequency',OperatingFrequency);

beaconPlat = phased.Platform('InitialPosition',[2000; 100; -20],...
  'Velocity',[1; 1; 0])


 %----------------% 
 
hydrophone1 = phased.IsotropicHydrophone('VoltageSensitivity',-150);
array1 = phased.ULA('Element',hydrophone1,...
  'NumElements',5,'ElementSpacing',propSpeed/OperatingFrequency/2,...
  'ArrayAxis','y');

arrayCollector1 = phased.Collector('Sensor',array1,...
  'PropagationSpeed',propSpeed,'OperatingFrequency',OperatingFrequency);

arrayPlat1 = phased.Platform('InitialPosition',[20; 10; -20],...
  'Velocity',[0; 0; 0]);

rx1 = phased.ReceiverPreamp(...
    'Gain',20,...
    'NoiseFigure',10,...
    'SampleRate',fs,...
    'SeedSource','Property',...
    'Seed',2007);

%------------------------%

hydrophone2 = phased.IsotropicHydrophone('VoltageSensitivity',-100);
array2 = phased.ULA('Element',hydrophone2,...
  'NumElements',5,'ElementSpacing',propSpeed/OperatingFrequency/2,...
  'ArrayAxis','y');

arrayCollector2 = phased.Collector('Sensor',array2,...
  'PropagationSpeed',propSpeed,'OperatingFrequency',OperatingFrequency);

arrayPlat2 = phased.Platform('InitialPosition',[20; 110; -20],...
  'Velocity',[0; 0; 0]);

rx2 = phased.ReceiverPreamp(...
    'Gain',20,...
    'NoiseFigure',10,...
    'SampleRate',fs,...
    'SeedSource','Property',...
    'Seed',2007);

x = wav();
numTransmits = 10;
rxsig1 = zeros(size(x,1),5,numTransmits);
rxsig2 = zeros(size(x,1),5,numTransmits);
for i = 1:numTransmits

  % Update array and acoustic beacon positions
  [pos_tx,vel_tx] = beaconPlat(1/prf);
  [pos_rx1,vel_rx1] = arrayPlat1(1/prf);
  [pos_rx2,vel_rx2] = arrayPlat2(1/prf);

  % Compute paths between the acoustic beacon and array
  [paths1,dop1,aloss1,rcvang1,srcang1] = ...
      isopaths(pos_tx,pos_rx1,vel_tx,vel_rx1,1/prf);
  
  [paths2,dop2,aloss2,rcvang2,srcang2] = ...
      isopaths(pos_tx,pos_rx2,vel_tx,vel_rx2,1/prf);

  % Propagate the acoustic beacon waveform
  tsig1 = projRadiator(x,srcang1);
  tsig2 = projRadiator(x,srcang2);
  rsig1 = channel1(tsig1,paths1,dop1,aloss1);
  rsig2 = channel2(tsig2,paths2,dop2,aloss2);
  
  % Collect the propagated signal
  rsig1 = arrayCollector1(rsig1,rcvang1);
  rsig2 = arrayCollector2(rsig2,rcvang2);
  
  % Store the received pulses
  rxsig1(:,:,i) = abs(rx1(rsig1));
  rxsig2(:,:,i) = abs(rx2(rsig2));
 
end

t = (0:length(x)-1)'/fs;
figure(2)
plot(t,rxsig1(:,end))
xlabel('Time (s)');
ylabel('Signal Amplitude (V)')
title("Received signal1")

t = (0:length(x)-1)'/fs;
figure(3)
plot(t,rxsig2(:,end))
xlabel('Time (s)');
ylabel('Signal Amplitude (V)')
title("Received signal2")

musicspatialspect1 = phased.MUSICEstimator('SensorArray',array1,...
        'PropagationSpeed',propSpeed,'OperatingFrequency',...
        OperatingFrequency,'ScanAngles',-90:0.1:90,'DOAOutputPort',true,...
        'NumSignalsSource','Property','NumSignals',1);

musicspatialspect2 = phased.MUSICEstimator('SensorArray',array2,...
        'PropagationSpeed',propSpeed,'OperatingFrequency',...
        OperatingFrequency,'ScanAngles',-90:0.1:90,'DOAOutputPort',true,...
        'NumSignalsSource','Property','NumSignals',1);
x=wav();
numTransmits = 500;
angPassive1 = zeros(numTransmits,1);
angPassive2 = zeros(numTransmits,1);

angAct1 = zeros(numTransmits,1);
angAct2 = zeros(numTransmits,1);

for i = 1:numTransmits

  % Update array and acoustic beacon positions
  [pos_tx,vel_tx] = beaconPlat(1/prf);
  [pos_rx1,vel_rx1] = arrayPlat1(1/prf);
  [pos_rx2,vel_rx2] = arrayPlat2(1/prf);

  % Compute paths between the acoustic beacon and array
  [paths1,dop1,aloss1,rcvang1,srcang1] = ...
      isopaths(pos_tx,pos_rx1,vel_tx,vel_rx1,1/prf);
  
  [paths2,dop2,aloss2,rcvang2,srcang2] = ...
      isopaths(pos_tx,pos_rx2,vel_tx,vel_rx2,1/prf);
  
  
  angAct1(i) = rcvang1(1,1);
  angAct2(i) = rcvang2(1,1);
  
  % Propagate the acoustic beacon waveform
  tsig1 = projRadiator(x,srcang1);
  tsig2 = projRadiator(x,srcang2);
  rsig1 = channel1(tsig1,paths1,dop1,aloss1);
  rsig2 = channel2(tsig2,paths2,dop2,aloss2);
  
  % Collect the propagated signal
  rsig1 = arrayCollector1(rsig1,rcvang1);
  rsig2 = arrayCollector2(rsig2,rcvang2);
  
  rxsig1 = rx1(rsig1);
  rxsig2 = rx2(rsig2);
 
  % Estimate the direction of arrival
  [~,angPassive1(i)] = musicspatialspect1(rxsig1);
  [~,angPassive2(i)] = musicspatialspect2(rxsig2);
  
end
figure(4)
plot([angPassive1 angAct1])
xlabel('Pulse Number')
ylabel('Arrival angle (degrees)')
legend('Estimated DOA','Actual DOA')
title("Direction/Angle of the target")

figure(5)
plot([angPassive2 angAct2])
xlabel('Pulse Number')
ylabel('Arrival angle (degrees)')
legend('Estimated DOA','Actual DOA')
title("Direction/Angle of the target")

angest1=(angPassive1(1));
angest2=(angPassive2(1));
L=100;
yest = L/(abs(tand(angest1)) + abs(tand(angest2)));
xest = yest*abs(tand(angest1));
zest = 0;
srcpos_est = [round(yest),round(xest),round(zest)]

N=5;
lst=zeros(N,3);
lst(1,1)=20; lst(1,2)=50;
lst(2,:)=[2046,50,10];
flag="false";

for i=1:N
    temp=lst(i,:);
    if(temp(1)==srcpos_est(1) && temp(2)==srcpos_est(2) && temp(3)==srcpos_est(3))
        disp("Not a target")
        flag="True";
    end
end

if(flag=="false")
    disp("Target Found")
end

% Constants for WGS84 ellipsoid
a = 6378137; % semi-major axis (equatorial radius) in meters
f = 1/298.257223563; % flattening factor

% Point A (known latitude and longitude)
latitude_A = 37.7749; % in degrees
longitude_A = -122.4194; % in degrees

% Cartesian coordinates of point B (x, y, z) in meters
x = srcpos_est(1); % meters
y = srcpos_est(2); % meters
z = srcpos_est(3); % meters

% Convert latitude and longitude of point A to radians
latitude_A_rad = deg2rad(latitude_A);
longitude_A_rad = deg2rad(longitude_A);

% Convert latitude and longitude of point A to ECEF coordinates
N = a / sqrt(1 - f * (2 - f) * sin(latitude_A_rad)^2);
X_A = (N + 0) * cos(latitude_A_rad) * cos(longitude_A_rad);
Y_A = (N + 0) * cos(latitude_A_rad) * sin(longitude_A_rad);
Z_A = (N * (1 - f^2) + 0) * sin(latitude_A_rad);

% Convert Cartesian coordinates of point B to ECEF coordinates
X_B = X_A + x;
Y_B = Y_A + y;
Z_B = Z_A + z;

% Convert ECEF coordinates of point B to latitude and longitude
p = sqrt(X_B^2 + Y_B^2);
longitude_B_rad = atan2(Y_B, X_B);
latitude_B_rad = atan2(Z_B, p);

% Convert latitude and longitude of point B to degrees
latitude_B = rad2deg(latitude_B_rad);
longitude_B = rad2deg(longitude_B_rad);

% Display latitude and longitude of point B
disp(['Latitude of point B: ', num2str(latitude_B), ' degrees']);
disp(['Longitude of point B: ', num2str(longitude_B), ' degrees']);




% figure(4)
% plotSpectrum(musicspatialspect)
% display(angPassive)