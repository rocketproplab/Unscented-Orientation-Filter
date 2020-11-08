function newCrossCorrelation = crossCorr2(possNewError,predError,...
    possExpMagMeas,predMagMeas,lambda,n)
    %
    % crossCorr2(possNewError,predError,possExpMagMeas,predMagMeas,...
    %   lambda,n)
    % 
    % finds the error to measurement cross correlation
    % Parameters:
    % predError is the predicted gyro error vector
    % possNewError are the possible new attitude error vectors
    % possExpMagMeas is the distribution of possible mag measurements based
    %   on the magnetic field at this location
    % predMagMeas is the predicted magnetometer measurement at the location
    % lambda and n are constants
    %
    % Result:
    % newCrossCorrelation is the correlation between error and measurement
    
    % following paper exactly
    CovSum = lambda*(possNewError(:,2*n+1) - predError)*transpose(...
        possExpMagMeas(:,2*n+1) - predMagMeas);
    
    for i = 1:(2*n)
        CovSum = CovSum + (possNewError(:,i) - predError)*transpose(...
            possExpMagMeas(:,i) - predMagMeas)/2;
    end
    
    newCrossCorrelation = CovSum/(n+lambda);
end