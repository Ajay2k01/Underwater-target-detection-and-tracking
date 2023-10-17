propSpeed = 1520;
channelDepth = 200;
OperatingFrequency = 37.5e3; 

isopaths = phased.IsoSpeedUnderwaterPaths('ChannelDepth',channelDepth,...
  'NumPathsSource','Property','NumPaths',10,'PropagationSpeed',propSpeed);

channel = phased.MultipathChannel('OperatingFrequency',OperatingFrequency);

prf = 1;
pulseWidth = 10e-3;
pulseBandwidth = 1/pulseWidth;
fs = 2*pulseBandwidth;
wav = phased.RectangularWaveform('PRF',prf,'PulseWidth',pulseWidth,...
  'SampleRate',fs);
channel.SampleRate = fs;

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

beaconPlat = phased.Platform('InitialPosition',[5000; 2000; -199],...
  'Velocity',[-5; 0; 0]);

hydrophone = phased.IsotropicHydrophone('VoltageSensitivity',-150);
array = phased.URA('Size',[10 5],'ElementSpacing',[0.3 0.5]);

arrayCollector = phased.Collector('Sensor',array,...
  'PropagationSpeed',propSpeed,'OperatingFrequency',OperatingFrequency);

arrayPlat = phased.Platform('InitialPosition',[0; 0; -10],...
  'Velocity',[0; 1; 0]);

rx = phased.ReceiverPreamp(...
    'Gain',20,...
    'NoiseFigure',10,...
    'SampleRate',fs*2,...
    'SeedSource','Property',...
    'Seed',2007);



musicspatialspect = phased.MUSICEstimator2D('SensorArray',array,...
        'PropagationSpeed',propSpeed,'OperatingFrequency',OperatingFrequency,...
        'AzimuthScanAngles',-45:45,'ElevationScanAngles',0:5,...
        'DOAOutputPort',true,...
        'NumSignalsSource','Property','NumSignals',1);
    
numTransmits = 50;
angPassive = zeros(numTransmits,2);
angAct = zeros(numTransmits,1);
angbroad=zeros(numTransmits,1);

for i = 1:numTransmits

  % Update array and acoustic beacon positions
  [pos_tx,vel_tx] = beaconPlat(1/prf);
  [pos_rx,vel_rx] = arrayPlat(1/prf);

  % Compute paths between acoustic beacon and the array
  [paths,dop,aloss,rcvang,srcang] = ...
      isopaths(pos_tx,pos_rx,vel_tx,vel_rx,1/prf);
  angAct(i) = rcvang(1,1);
  
  % Propagate the acoustic beacon waveform
  tsig = projRadiator(x,srcang);
  rsig = channel(tsig,paths,dop,aloss);
  
  % Collect the propagated signal
  rsig = arrayCollector(rsig,rcvang);
  
  rxsig = rx(rsig);
 
  % Estimate the direction of arrival
  [~,angPassive(i,1:2)] = musicspatialspect(rxsig);
  angbroad(i)=az2broadside(angPassive(i,1),angPassive(i,2));
  
end



  
% figure(2);
% t = (0:length(x)-1)'/fs;
% plot(t,rxsig(:,end))
% xlabel('Time (s)');
% ylabel('Signal Amplitude (V)')

figure(2)
plot([angbroad angAct])
xlabel('Pulse Number')
ylabel('Arrival angle (degrees)')
legend('Estimated DOA','Actual DOA')
title("Direction/Angle of the target")

figure(3)
plotSpectrum(musicspatialspect)

display(angPassive)
% bsAngle=angPassive(1);
% numElements=5;
% elementSpacing=propSpeed/OperatingFrequency/2;
% lambda= 1520/OperatingFrequency;
% % Calculate the estimated azimuthal angle
% azEst = atan2(sin(bsAngle), cos(bsAngle)*sin(pi/2 - (0:numElements-1)*pi*elementSpacing/lambda));
% 
% % Calculate the estimated elevation angle
% elEst = atan2(sin(bsAngle)*cos(pi/2 - (0:numElements-1)*pi*elementSpacing/lambda), cos(bsAngle));
% 
% display(azEst);display(elEst)


% % Calculate the array response pattern over a range of angles
% azRange = -90:0.5:90; % Range of azimuth angles to evaluate
% elRange = -90:0.5:90; % Range of elevation angles to evaluate
% [azGrid,elGrid] = meshgrid(azRange,elRange);
% 
% 
% % Plot the 3D graph of azimuth, elevation, and power
% figure(5);
% surf(azRange,elRange,musicspatialspect);
% xlabel('Azimuth Angle (degrees)');
% ylabel('Elevation Angle (degrees)');