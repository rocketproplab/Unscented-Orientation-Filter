function quat = kalmanArrayMult(q,p)
    % Ep. (18) using Eq. (16a)
    % kalmanArrayMult multiplies two quaternions using
    %   the array method
    % q and p are both input quaternions (column 4vecs)
    %   the fourth component is the scalar one
    % quat is the quaternion such that:
    % quat = q*p
    
    Eps = [p(4,1)*eye(3)+crossMatrix(p(1:3,1)); -(p(1:3,1))'];
    
    quat = [Eps, p]*q;
%    quat = multQuat(q,p);
end
