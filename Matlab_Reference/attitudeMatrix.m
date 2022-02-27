function Aq = attitudeMatrix(quat)
    
    % TODO: consider symbolically solving for Aq for speed
	% Generate an Attitude Matrix for the given quaternion.

    % Eq. (16a)
    Eps = [quat(4,1)*eye(3)+crossMatrix(quat(1:3,1)); -(quat(1:3,1))'];
    
    % Eq. (16b)
    Psi = [quat(4,1)*eye(3)-crossMatrix(quat(1:3,1)); -(quat(1:3,1))'];
    
    % Eq.  (15)
    Aq = transpose(Eps)*Psi;
end