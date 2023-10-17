[trueState, time, fig1] = helperGenerateTruthData;

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

dist1 = zeros(1,numSteps);
estPos1 = zeros(3,numSteps);
for i = 2:size(measPos,2)
    predict(cvekf, dt);
    dist1(i) = distance(cvekf,truePos(:,i)); % Distance from true position
    estPos1(:,i) = positionSelector * correct(cvekf, measPos(:,i));
end

cvekf = trackingEKF(@constvel, @cvmeas, initialState, ...
    'StateTransitionJacobianFcn', @constveljac, ...
    'MeasurementJacobianFcn', @cvmeasjac, ...
    'StateCovariance', initialCovariance, ...
    'HasAdditiveProcessNoise', false, ...
    'ProcessNoise', diag([50,50,1]));

dist2 = zeros(1,numSteps);
estPos2 = zeros(3,numSteps);
for i = 2:size(measPos,2)
    predict(cvekf, dt);
    dist2(i) = distance(cvekf,truePos(:,i)); % Distance from true position
    estPos2(:,i) = positionSelector * correct(cvekf, measPos(:,i));
end

figure(fig1)
plot(estPos1(1,:),estPos1(2,:),'.g','DisplayName','CV Low PN')
title('True and Estimated Positions')
axis([5000 8000 -500 2500])
hold on
plot(estPos2(1,:),estPos2(2,:),'.b','DisplayName','CV High PN')

axis([5000 8000 -500 2500])


fig2 = figure;
hold on
plot((1:numSteps)*dt,dist1,'g','DisplayName', 'CV Low PN')
title('Normalized Distance From Estimated Position to True Position')
xlabel('Time (s)')
ylabel('Normalized Distance')
legend

figure(fig2)
plot((1:numSteps)*dt,dist2,'c','DisplayName', 'CV High PN')
axis([0 100 0 50])


imm = trackingIMM('TransitionProbabilities', 0.99); % The default IMM has all three models
% Initialize the state and state covariance in terms of the first model
initialize(imm, initialState, initialCovariance);

dist3 = zeros(1,numSteps);
estPos3 = zeros(3,numSteps);
modelProbs = zeros(3,numSteps);
modelProbs(:,1) = imm.ModelProbabilities;
for i = 2:size(measPos,2)
    predict(imm, dt);
    dist3(i) = distance(imm,truePos(:,i)); % Distance from true position
    estPos3(:,i) = positionSelector * correct(imm, measPos(:,i));
    modelProbs(:,i) = imm.ModelProbabilities;
end
figure(fig1)
plot(estPos3(1,:),estPos3(2,:),'.m','DisplayName','IMM')

figure(fig2)
hold on
plot((1:numSteps)*dt,dist3,'m','DisplayName', 'IMM')
axis([0 100 0 50])

figure
plot((1:numSteps)*dt, modelProbs)
title('Model Probabilities vs. Time')
xlabel('Time (s)')
ylabel('Model Probabilities')
legend('IMM-CV','IMM-CA','IMM-CT')
% Return the RNG to its previous state
rng(s)

