function possNewError = newChis(possNewQuats,possBias,n,f,a)
    
    % newChis(possNewQuats,possBias,n,f,a)
    % finds possible new errors in the angular velocity measurements
    %
    % Parameters:
    % possNewQuats are the possible orientations of the spacecraft after
    %   the most recent rotation measured by the gyro
    % possBias are possible bias values in the gyro measurement
    % n,f,a are constants and fine tuning values
    %
    % Result:
    % possNewError are the possible new attitude error vectors
    
    % Eq. (36)
    % Find the propogated error quaternions
    % dqK1(:,(2*n+1)) (which is the dqK1(0) quaternion) should come
    %   out as an identity quaternion
    dqK1 = zeros(4,2*n+1);
    for i=1:(2*n+1)
        dqK1(:,i) = kalmanArrayMult(possNewQuats(:,i), kalmanArrayInv(...
			possNewQuats(:,2*n+1)));
    end
    
    % Eq. (37)
    % Find the attitude error part of the propogated Chi values
    % Don't use dqK1(0), since ChiK1(0) is the zero vector
    possNewError = zeros(3,2*n+1);
    for i=1:(2*n)
        possNewError(1:3,i) = f*dqK1(1:3,i)/(a+dqK1(4,i));
    end
    
    % Add ChiK1(0) to the end
    possNewError(1:3,2*n+1) = [0;0;0];
    
    % Eq. (38)
    % Add the gyro bias part of the Chi values to the bottom
    % Bias is the same in different steps, so this is just the previous
    %   bias.
    possNewError = [possNewError;possBias];
end