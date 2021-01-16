function quat = kalmanArrayMult(q,p)
    
    % kalmanArrayMult multiplies two quaternions using
    %   the array method
    % q and p are both input quaternions (column 4vecs)
    %   the fourth component is the scalar cosine one
    % quat is the quaternion such that:
    % quat = q*p
    
	% Eq. (18) using Eq. (16a)
    Eps = [p(4,1)*eye(3)+crossMatrix(p(1:3,1)); -(p(1:3,1))'];
    
    quat = [Eps, p]*q;
end
