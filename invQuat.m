function newQuat = invQuat(quat1)
    % invQuat(quat1)
    %
    % Finds the inverse of quat1
    % quat1 is the quaternion whose conjugate should be found
    % This uses the quaternion convention with the scalar cosine term 4th
    
    newQuat = conjQuat(quat1)/(norm(quat1)^2);
end