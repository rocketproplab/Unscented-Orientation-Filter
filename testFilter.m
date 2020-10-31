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
    clear functions;
    
    % Define initial attitude estimate (actual attitude in idealPath)
    attitudeQuat = [0;0;0;1];
    
    % Define attitude error and bias error components of covariance
    att_err = (pi/360)^2;
    bias_err = (pi/(3.24e6))^2;
    covariance = diag([att_err,att_err,att_err,bias_err,bias_err,...
        bias_err]);
    
    % Define gyro bias estimate
    gyrobias = [0;0;0];%0.5*(pi/6.48)*(1e-5)*[1;1;1];
    
    % Run Simulation
    [v2errQs,v1errQs] = runFilter(attitudeQuat,covariance,gyrobias);
    
    % graph stuff in degrees for easy reading
    v1theta = zeros(1,size(v1errQs,2));
    for i=1:size(v1errQs,2)
        v1theta(1,i) = 2*atand(norm(v1errQs(1:3,i))/v1errQs(4,i));
        %v1theta(1,i) = abs(v1err(4,i));
    end
    
    % graph stuff in degrees for easy reading
    v2theta = zeros(1,size(v2errQs,2));
    for i=1:size(v2errQs,2)
        v2theta(1,i) = 2*atand(norm(v2errQs(1:3,i))/v2errQs(4,i));
        %v2theta(1,i) = abs(v2errorQuats(4,i));
    end
    
    % time
    iter = max(size(v1errQs,2),size(v2errQs,2));
    t = linspace(1,10*iter,iter);
    
    figure;
    plot(t,v1theta);
    title('Naive Integration');
    %hold on
    figure;
    plot(t,v2theta);
    title('USQUE');
    %legend('justQuats','USQUE');
end
    
        