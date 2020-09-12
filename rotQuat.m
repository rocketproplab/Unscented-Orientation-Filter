function quat = rotQuat(theta, v)
    % quat(theta, v)
    % 
    % This function is adapted to the Crassadis Markley quaternion
    %   convention, with the fourth term being the scalar term
    % Finds a rotation quaternion
    %   from a total rotation angle
    %   and a rotation vector
    % theta is an angle in degrees
    % v is a vector with 3 entries
    
    quat = [0;0;0;0];
    quat(4) = cosd(theta/2);
    quat(1) = v(1)*sind(theta/2);
    quat(2) = v(2)*sind(theta/2);
    quat(3) = v(3)*sind(theta/2);
end