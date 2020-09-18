function [gyroMeas] = readGyro(idealAngV)
    % 
    % readGyro
    %
    % This function is intended to simulate gyro measurements, and is used
    %   to test the filter. In the Crassidis and Markley simulations, the
    %   gyro is modeled as having:
    sigma_bias = 3.1623e-10; % radians per (3/2) second,
    % sigma_bias = 0.018119*1e-6; % degrees per (3/2) second,
    
    sigma_noise = 0.31623e-6; % radians per (1/2) second,
    % sigma_noise = 18.119*1e-6; % degrees per (1/2) second
    
    % and an initial bias of 0.1 degrees per hour on each axis.
    % The bias is updated with a zero mean Gaussian white-noise process
    %   with standard deviation equal to sigma_bias
    % Besides bias, the gyro measurements also have zero mean Gaussian 
    %   white-noise added, with standard deviation equal to sigma_noise
    
    % For an 9 hour simulation, with measurements every 10 seconds,
    %   generate 3240 simulated measurements.
    % Assume the spacecraft makes a full rotation and a full orbit every 90
    %   minutes. Then the spacecraft makes 6 full rotations in this
    %   simulation.
    
    % DO STUFF IN RADIANS
    
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
    