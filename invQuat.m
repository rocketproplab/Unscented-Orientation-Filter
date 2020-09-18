function newQuat = invQuat(quat1)
    % invQuat(quat1)
    %
    % Finds the inverse of quat1
    % quat1 is a vector with 4 entries
    % quat1 is the quaternion whose conjugate should be found
    % cosine term 4th
    
    newQuat = conjQuat(quat1)/(norm(quat1)^2);
end