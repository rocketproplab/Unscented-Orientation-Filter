function quat = kalmanArrayMult2(q,p)
    % Ep. (18) using Eq. (16b)
    % kalmanArrayMult multiplies two quaternions using
    %   the array method
    % q and p are both input quaternions (column 4vecs)
    %   the fourth component is the scalar one
    % quat is the quaternion such that:
    %   quat = q*p
    
    Psi = [q(4,1)*eye(3)-crossMatrix(q(1:3,1)); -(q(1:3,1))'];
    
    quat = [Psi, q]*p;
end