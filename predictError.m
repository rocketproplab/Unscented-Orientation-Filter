function [predError, predCov] = predictError(lambda, possNewError, noiseCov, n)
    %
    % predictError(lambda,possNewError,noiseCov,n)
    % predict the gyromeasurement error and the new covariance for this step.
    %
    % Parameters:
    % possNewError are the possible new attitude error vectors
    % noiseCov is the constant noise covariance QbarK
    % lambda and n are a fine tuning parameter and a constant
    %
    % Results:
    % predError is the predicted gyro error vector
    % predCov is the predicted new covariance
    
    % Now that we have our propagated Chi values  we can find our 
    %   predicted values of interest. Eq. (7) is for predicted mean
    %   error (pre-update).
    predError = (lambda*possNewError(:,2*n+1) ...
        + 0.5*sum(possNewError(:,1:2*n),2))/(n+lambda);
        
    % Eq. (8)
    % Find the predicted covariance (pre-update).
    P_in = possNewError(:,1:(2*n)) - repmat(predError,1,2*n);
        
    predCov = (lambda*(possNewError(:,2*n+1)-predError)* ...
        (transpose(possNewError(:,2*n+1)-predError)) ...
        + 0.5*P_in*transpose(P_in))/(n+lambda) + noiseCov;
end
        