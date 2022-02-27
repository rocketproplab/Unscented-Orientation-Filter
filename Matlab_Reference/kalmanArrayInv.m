function invQuat = kalmanArrayInv(quat)
    %
    % kalmanArrayInv finds the inverse of a quaternion
    % quat is the quaternion whose inverse should be found
    % This uses the quaternion convention with the scalar cosine term 4th
    
    invQuat = ([-quat(1:3,1); quat(4,1)]/(norm(quat)^2));
end