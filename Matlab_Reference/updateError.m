function [error,covariance] = updateError(magMeas,predMagMeas,...
    newCrossCorrelation,newInnovationCov,predError,predCov)
    %
    % updateError(magMeas,predMagMeas,newCrossCorrelation,...
    %       newInnovationCov,predError,predCov)
    % Finds the error in the orientation based on how the magnetometer 
    %   measurement compares to the predicted magnetometer measurement.
    %   Also finds the covariance using the error to measurement
    %   correlation, the innovation-innovation covariance, and the
    %   predicted covariance.
    %
    % Parameters:
    % magMeas is the magnetometer measurement
    % predMagMeas is the predicted magnetometer measurement at the location
    % newCrossCorrelation is the correlation between error and magMeas
    % newInnovationCov is the covariance between the innovation and itself
    % predError is the predicted gyro error vector
    % predCov is the predicted covariance
    %
    % Results:
    % error is the approximated error vector of this orientation
    %   calculation
    % covariance is the final covariance for this iteration
    %
    % Eq. (3)
	% Find the innovation (measurement minus expected measurement)
	vK1 = magMeas - predMagMeas;
        
	% Eq. (4)
	% Find the gain
	KK1 = newCrossCorrelation/(newInnovationCov);
        
	% Eq. (2a)
	% Find the updated, propagated state (mean error vector)
	error = predError + KK1*vK1;
        
	% Eq. (2b)
	% Find the updated, propagated covariance
	% This is one of the important results of each loop, and will be
	%   used in the next loop.
	covariance = predCov - KK1*newInnovationCov*transpose(KK1);
end