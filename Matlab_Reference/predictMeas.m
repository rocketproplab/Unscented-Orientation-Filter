function predMagMeas = predictMeas(lambda,possExpMagMeas,n)
    % 
    % predictMeas(lambda,possExpMagMeas,n)
    %
    % predict the measurement the magnetometer will make based on the
    %   possible magnetometer measurements
    % 
    % Parameters:
    % possExpMagMeas are the possible magnetometer measurements that you 
    %   expect
    % lambda and n are fine tuning and a constant
    %
    % Results:
    % predMagMeas is the predicted magnetometer measurement
    
    % Eq. (9)
	% Find the mean expected observation vector
	predMagMeas = (lambda*possExpMagMeas(:,2*n+1)+0.5*sum(...
        possExpMagMeas(:,1:(2*n)),2))/(n+lambda);
end