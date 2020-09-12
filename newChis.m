function ChiK1 = newChis(qK1,chiBias,n,f,a)
    
    
    % Eq. (36)
    % Find the propogated error quaternions
    % dqK1(:,(2*n+1)) (which is the dqK1(0) quaternion) should come
    %   out as an identity quaternion
    dqK1 = zeros(4,2*n+1);
    for i=1:(2*n+1)
        dqK1(:,i) = kalmanArrayMult(qK1(:,i), ...
            kalmanArrayInv(qK1(:,2*n+1)));
    end
    
    % Eq. (37)
    % Find the attitude error part of the propogated Chi values
    % Don't use dqK1(0), since ChiK1(0) is the zero vector
    ChiK1 = zeros(3,2*n+1);
    for i=1:(2*n)
        ChiK1(1:3,i) = f*dqK1(1:3,i)/(a+dqK1(4,i));
    end
    
    % Add ChiK1(0) to the end
    ChiK1(1:3,2*n+1) = [0;0;0];
    
    % Eq. (38)
    % Add the gyro bias part of the Chi values to the bottom
    % Bias is the same in different steps, so this is just the previous
    %   bias.
    ChiK1 = [ChiK1;chiBias];
end