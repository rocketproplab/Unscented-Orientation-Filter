function newInnovationCov = innovationCov2(possExpMagMeas,predMagMeas,...
    lambda,sigma_mag,n)
    %
    % innovationCov2(possExpMagMeas,predMagMeas,lambda,sigma_mag,n)
    % 
    % This function calculates the innocation covariance
    %
    % Parameters:
    % possExpMagMeas is the distribution of possible mag measurements based
    %   on the magnetic field at this location
    % predMagMeas is the predicted magnetometer measurement at the location
    % sigma_mag is the standard deviation of magnetomere noise
    % lambda and n are constants
    %
    % Results:
    % newInnovationCov is the innovation covariance
    %
    % Stricltly following research paper
    CovSum = lambda*(possExpMagMeas(:,2*n+1) - predMagMeas)*...
        transpose(possExpMagMeas(:,2*n+1) - predMagMeas);
    
    for i = 1:(2*n)
        CovSum = CovSum + (possExpMagMeas(:,i) - predMagMeas)*...
            transpose(possExpMagMeas(:,i) - predMagMeas)/2;
    end
    
    PK1yy = CovSum/(n+lambda);
    
    newInnovationCov = PK1yy + (sigma_mag^2)*eye(3);
end