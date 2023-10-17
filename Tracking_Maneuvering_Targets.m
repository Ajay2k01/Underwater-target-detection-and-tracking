[trueState, time, fig1, seg1,seg2] = helperGenerateTruthData;

dt = diff(time(1:2));
numSteps = numel(time);
figure(1)



% Set the RNG seed for repeatable results
s = rng;
rng(2018);
positionSelector = [1 0 0 0 0 0;0 0 1 0 0 0;0 0 0 0 1 0]; % Position from state
truePos = positionSelector * trueState;
measNoise = randn(size(truePos));
measPos = truePos + measNoise;

initialState = positionSelector' * measPos(:,1);
initialCovariance = diag([1,1e4,1,1e4,1,1e4]); % Velocity is not measured
cvekf = trackingEKF(@constvel, @cvmeas, initialState, ...
    'StateTransitionJacobianFcn', @constveljac, ...
    'MeasurementJacobianFcn', @cvmeasjac, ...
    'StateCovariance', initialCovariance, ...
    'HasAdditiveProcessNoise', false, ...
    'ProcessNoise', eye(3));

dist = zeros(1,numSteps);
estPos = zeros(3,numSteps);
for i = 2:size(measPos,2)
    predict(cvekf, dt);
    dist(i) = distance(cvekf,truePos(:,i)); % Distance from true position
    estPos(:,i) = positionSelector * correct(cvekf, measPos(:,i));
end

figure(fig1)
plot(estPos(1,:),estPos(2,:),'.g','DisplayName','CV Low PN')
title('True and Estimated Positions')
axis([5000 8000 -500 2500])
display(diff(time(1:2)))

