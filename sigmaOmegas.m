function possAngV = sigmaOmegas(gyroMeas,error,possBias,n)
    
    % sigmaOmegas(gyroMeas,error,possBias,n)
    % finds possible true angular velocities of the spacecraft based on the
    %   measured angular velocity and the possible true orientations
    %
    % Parameters:
    % gyroMeas is the angular velocity measured by the spacecraft
    % error is the best estimate of the error in gyroMeas compared to the
    %   true angular velocity
    % possBias are possible bias values in gyroMeas compared to the true
    %   angular velocity
    % n is a constant representing half of (the number of possible values
    %   -1)
    %
    % Results:
    % possAngV is the possible true angular velocities
    
    % Eq. (35)
    % Create a sigma estimated angular velocity array using
    %   Chi bias sigma vectors
    possAngV = zeros(3,2*n+1);
    for i=1:(2*n)
        possAngV(:,i) = gyroMeas-possBias(:,i);
    end
    
    % Add the omegaK(0) three vector to the end of the array. The omegaK
    %   array now has 13 columns.
    possAngV(:,2*n+1) = gyroMeas - error(4:6,1);
end