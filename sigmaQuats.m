function [qK,chiBias] = sigmaQuats(lambda,PplusK,QbarK,stateV,a,f, ...
    attitudeQuat,n)

    % Eq. (5)a 
    % Generate sigma vectors from +- the Cholesky decomposition of a 
    %   weighted sum of the postupdate covariance and a process noise 
    %   related covariance.
    Sigma = chol((6+lambda)*(PplusK + QbarK));
    Sigma = [Sigma, -Sigma];
    
    % Eq. (5)b,c
    % Find Chi vectors, by adding the estimated postupdate state to each 
    %   sigma vector. 
    % These Chi vectors are basically error vectors, all of which together 
    %   have the distribution described by PlusK and QbarK
    Chi = Sigma + repmat(stateV,1,2*n);
    Chi = [Chi,stateV];
    
    
    % Return the bottom three components of each Chi vector, as the bias
    %   component. This component does not change between propogations, and
    %   will be used in Eq. (35) in sigmaOmegas, and in Eq. (38) in the
    %   newChis() function.
    chiBias = Chi(4:6,:);
    
    
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
    qK = zeros(4,2*n+1);
    for i=1:(2*n)
        qK(:,i) = kalmanArrayMult(dqK(:,i),attitudeQuat);
    end
    
    % Add the qK(0) quaternion, equal to the current quaternion estimate,
    %   to the end of the array. The qK array now has 13 columns.
    qK(:,2*n+1) = attitudeQuat;
end
    