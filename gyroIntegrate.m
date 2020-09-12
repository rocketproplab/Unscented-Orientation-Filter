function newQuat = gyroIntegrate(initialQ, gyroData, deltaTime)
    % gyroIntegrate(initialQ, gyroData, deltaTime)
    %
    % This function is adapted to the Crassadis Markley quaternion
    %   convention, with the fourth term being the scalar term
    % Finds the new orientation quaternion from the
    %   the initial orientation quaternion, the
    %   rotation vector from the gyroscope, and the
    %   time delay of the rotation vector
    % initialQ is a vector with 4 entries
    % gyroData is a vector with 3 entries
    % deltaTime is a number
    
    deltaQuat = rotQuat(deltaTime*norm(gyroData), gyroData/norm(gyroData));
    newQuat = multQuat(initialQ,deltaQuat);
end
    