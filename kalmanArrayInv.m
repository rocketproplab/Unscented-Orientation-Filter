function invQuat = kalmanArrayInv(quat)
    %
    % kalmanArrayInv finds the inverse of a quaternion
    % quat is a quaternion 4-vector, with the scalar
    %   term as the last term
    
    invQuat = [-quat(1:3,1); quat(4,1)];
end