function [] = testFilter()
    %
    %
    % HOW to test: The paper lists simulation gyro and magnetometer
    %   properties, including a gyro bias of 0.1 deg/h on each axis.
    % Test 1: no initial attitude errors, but initial bias estimate set to
    %   zeros. Small covariace
    % Test 2: large initial attitude errors, large initial attitude
    %   covariance.
    % Test 3: same attitude errors, estimated bias on one axis roughly half
    %   of last initial attitude covariance, attitude covariance likewise
    %   roughlt halfs of last initial attitude covariance.
    %
    attitudeQuat = [0;0;0;1];
    covariance = diag([0.25,0.25,0.25,0.04,0.04,0.04]);
    gyrobias = [0;0;0];
    gyroDt = 10;
    
    errorQuats = runFilter(attitudeQuat,covariance,gyrobias,gyroDt);
    
    % graph stuff in degrees for easy reading
    theta = zeros(1,size(errorQuats,2));
    t = zeros(1,size(errorQuats,2));
    for i=1:size(errorQuats,2)
        t(1,i) = 10*(i-1);
        theta(1,i) = 2*atand(norm(errorQuats(1:3,i))/errorQuats(4,i));
        %theta(1,i) = abs(errorQuats(4,i));
    end
    plot(t,theta);
    
    clear;
end
    
        