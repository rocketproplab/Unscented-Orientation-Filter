function quat = multQuat(q,p)
    % multQuat(quat1,quat2)
    %
    % Multiplies two quaternions
    % q is a vector with 4 entries
    % p is a vector with 4 entries
    % cosine term 4th
    
    quat = [0;0;0;0];
    
    quat(4,1) = q(4)*p(4) - q(1)*p(1) - q(2)*p(2) - q(3)*p(3);
    quat(1,1) = q(4)*p(1) + q(1)*p(4) + q(2)*p(3) - q(3)*p(2);
    quat(2,1) = q(4)*p(2) - q(1)*p(3) + q(2)*p(4) + q(3)*p(1);
    quat(3,1) = q(4)*p(3) + q(1)*p(2) - q(2)*p(1) + q(3)*p(4);
end