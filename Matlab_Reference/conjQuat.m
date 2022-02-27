function newQuat = conjQuat(quat)
    % conjQuat(quat1)
    %
    % Finds the conjugate of a quaternion
    % quat is the quaternion whose conjugate should be found
    % This uses the quaternion convention with the scalar cosine term 4th
    
    newQuat = [-quat(1:3);quat(4)];
end