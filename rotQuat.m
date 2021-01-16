function quat = rotQuat(theta, v)
    % quat(theta, v)
    
	% Finds a rotation quaternion from a total rotation angle and a 
	%	rotation vector
    % This function is adapted to the Crassadis Markley quaternion
    %   convention, with the fourth term being the scalar cosine term
    % theta is an angle in RADIANS
    % v is a vector with 3 entries
    
    % normalize v
    v = v/norm(v);
    
    quat = [0;0;0;0];
    quat(4,1) = cos(theta/2);
    quat(1,1) = v(1)*sin(theta/2);
    quat(2,1) = v(2)*sin(theta/2);
    quat(3,1) = v(3)*sin(theta/2);
end