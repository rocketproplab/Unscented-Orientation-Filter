function newQuat = gyroIntegrate(initialQ,gyroMeas,gyroDt)
    % gyroIntegrate(initialQ, gyroData, deltaTime)
    %
    % This function finds the new orientation quaternion from an initial 
    %   orientation quaternion, the rotation vector from the gyroscope, and
    %   the time delay of the rotations. 
    % It is adapted to Crassadis and Markley's quaternion convention, with 
    %   the fourth term being the scalar cosine term.
    %
    % Parameters:
    % initialQ is a vector with 4 entries (quaternion)
    % gyroData is a vector with 3 entries
    % deltaTime is a scalar
    %
    % Results:
    % newQuat is a new orientation quaternion
    
    deltaQuat = rotQuat(gyroDt*norm(gyroMeas), gyroMeas/norm(gyroMeas));
    newQuat = multQuat(initialQ,deltaQuat);
end
    