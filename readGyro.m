function [gyroMeas] = readGyro(idealAngV)
    % 
    % readGyro(idealAngV)
    %
    % This function is simulates gyro measurements, with noise and bias,
    %   given the true angular velocity of the spacecraft.
    %
    % Parameters:
    % idealAngV is the true angular velocity of the spacecraft
    %
    % Results:
    % gyroMeas is the simulated gyroscope measurement
    
    
    % True gyro bias standard deviation in radians per second^(3/2)
    sigma_bias = 3.1623e-10;
    
    % True gyro noise standard deviation in radians per second^(1/2)
    sigma_noise = 0.31623e-6;
    
    
    % The bias is updated with a zero mean Gaussian white-noise process
    %   with standard deviation equal to sigma_bias
    % Besides bias, the gyro measurements also have zero mean Gaussian 
    %   white-noise added, with standard deviation equal to sigma_noise
    
    
    % DO STUFF IN RADIANS AND SECONDS
    
    persistent bias
    if isempty(bias)
        
        % calculate bias per time step, initialize it here
        bias = (pi/6.48)*(1e-5)*[1;1;1]; % rad/s
    end
    
    % Make sure that the standard deviation values are for time deltas
    %   corresponding to your measurement time deltas. Otherwise, calculate
    %   the standard deviation of the mean (divide the standard deviation
    %   by the number of measurements per delta, to get the new standard
    %   deviation). For the research paper simulation examples I assume
    %   this is the case.
    bias = bias + (sigma_bias)*randn(3,1);
    noise = (sigma_noise)*randn(3,1);
    
    gyroMeas = idealAngV + bias + noise;
end
    