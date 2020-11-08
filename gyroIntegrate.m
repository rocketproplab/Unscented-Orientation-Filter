function newQuat = gyroIntegrate(initialQ,gyroMeas,gyroDt)
    % gyroIntegrate(initialQ, gyroData, deltaTime)
    %
    % This function finds the new orientation quaternion from an initial 
    %   orientation quaternion, the rotation vector from the gyroscope, and
    %   the time delay of the rotationis 
    % It is adapted to the Crassadis Markley quaternion convention, with 
    %   the fourth term being the scalar term
    %
    % Parameters:
    % initialQ is a vector with 4 entries
    % gyroData is a vector with 3 entries
    % deltaTime is a number
    %
    % Results:
    % newQuat is a new orientation quaternion
    
    deltaQuat = rotQuat(gyroDt*norm(gyroMeas), gyroMeas/norm(gyroMeas));
    newQuat = multQuat(initialQ,deltaQuat);
end
    