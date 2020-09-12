function QbarK = processNoise(gyroDt,sigma_v,sigma_u)
    %
    % Find the matrix related to the discrete process covariance
    % Eq. (42)
    % QbarK does is only based on the gyro bias and noise properties and
    %   thus does not change throughout the operation of the filter
    QbarK = (gyroDt/2)*[(sigma_v^2 - (1/6)*sigma_u^2*gyroDt^(2))*eye(3),...
        zeros(3); zeros(3), (sigma_u^2)*eye(3)];
end