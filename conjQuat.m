function newQuat = conjQuat(quat1)
    % conjQuat(quat1)
    %
    % Finds the conjugate of a quaternion
    % quat1 is a vector with 4 entries
    % quat1 is the quaternion whose conjugate should be found
    % cosine term 4th
    
    newQuat(1) = -quat1(1);
    newQuat(2) = -quat1(2);
    newQuat(3) = -quat1(3);
    newQuat(4) = quat1(4);