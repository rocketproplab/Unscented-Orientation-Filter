function [possQuats,possBias] = sigmaQuats(lambda,covariance,noiseCov,...
    error,a,f,attitudeQuat,n)

    % sigmaQuats(lambda,ovariance,noiseCov,error,a,f,attitudeQuat,n)
    %
    % This function find possible orientation quaternions from the best
    %   estimate of the last filter iteration, and the covariances.
    %
    % Parameters:
    % lambda, a, and f are fine tuning values
    % covariance and noiseCov are PplusK and QbarK in the research paper
    % error is a angular velocity error estimate from the last iteration
    %
    % Results:
    % possQuats are possible current orientation quaternions (before the
    %   most recent rotation). They represent the distribution of possible
    %   orientations.
    % possBias are possible bias values of the gyroscope

    % Eq. (5)a 
    % Generate sigma vectors from +- the Cholesky decomposition of a 
    %   weighted sum of the postupdate covariance and a process noise 
    %   related covariance.
    Sigma = transpose(chol((6+lambda)*(covariance + noiseCov)));
    Sigma = [Sigma, -Sigma];
    
    % Eq. (5)b,c
    % Find Chi vectors, by adding the estimated postupdate state to each 
    %   sigma vector. 
    % These Chi vectors are basically error vectors, all of which together 
    %   have the distribution described by PlusK and QbarK
    Chi = Sigma + repmat(error,1,2*n);
    Chi = [Chi,error];
    
    
    % Return the bottom three components of each Chi vector, as the bias
    %   component. This component does not change between propogations, and
    %   will be used in Eq. (35) in sigmaOmegas, and in Eq. (38) in the
    %   newChis() function.
    possBias = Chi(4:6,:);
    
    
    % Eq. (33)
    % At this point Chi has 12 columns, no Chi(0)
    % For each chi vector, find a dq vector which represents the error
    %   vector in quaternion form (used for multiplication)
    dqK = zeros(4,2*n);
    for i=1:(2*n)
        dqK(4,i) = (-a*(norm(Chi(1:3,i)))^2 ...
        + f*sqrt(f^2 + (1-a^2)*(norm(Chi(1:3,i)))^2))/(f^2 ...
        + (norm(Chi(1:3,i)))^2);
        
        dqK(1:3,i) = ((a+dqK(4,i))/f)*Chi(1:3,i);
    end
    
    % Eq. (32)
    % For each error quaternion, generate a qK sigma quaternion
    % Do this by multiplying by the state quaternion
    % Together, these represent the distribution of likely orientation
    %   quaternions
    possQuats = zeros(4,2*n+1);
    for i=1:(2*n)
        possQuats(:,i) = kalmanArrayMult(dqK(:,i),attitudeQuat);
    end
    
    % Add the qK(0) quaternion, equal to the current quaternion estimate,
    %   to the end of the array. The qK array now has 13 columns.
    possQuats(:,2*n+1) = attitudeQuat;
end
    