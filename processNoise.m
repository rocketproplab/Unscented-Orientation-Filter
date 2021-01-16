function noiseCov = processNoise(gyroDt,sigma_noise,sigma_bias)
    %
    % Find the matrix related to the discrete process covariance
    % Eq. (42)
    % QbarK does is only based on the gyro bias and noise properties and
    %   thus does not change throughout the operation of the filter
    noiseCov = (gyroDt/2)*[(sigma_noise^2 - (1/6)*sigma_bias^2*...
		gyroDt^(2))*eye(3),zeros(3); zeros(3), (sigma_bias^2)*eye(3)];
end