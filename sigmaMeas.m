function possExpMagMeas = sigmaMeas(possNewQuats,magField,n)
    %
    % sigmaMeas(possNewQuats,magField,n)
    %
    % Parameters:
    % possNewQuats are the possible orientation quaternions after the most
    %   recent rotation
    % magField is the magnetic field vector at this location from the World
    %   Magnetic Model
    % n is a constant
    %
    % Results:
    % possExpMagMeas are the possible expected magnetometer measurements.
    %   They represent the distribution of magnetometer measurements we
    %   think we can get based on what we think is the magnetic field at
    %   this location and our possible orientations after the last
    %   rotation.
    
    % Eq. (43)
    % For each sigma quaternion, generate an expected measurement
    % If you're adding a measurement, and you have an expected vector
    %   (in the world frame) for the measurement, you can add it here.
    %   Just add another three components to the bottom of each gammaK1
    %   column vector, and update the size you are initializing the 
	%   gammaK1 array to.
	possExpMagMeas = zeros(3,2*n+1);
    for i=1:(2*n+1)
        Aq = attitudeMatrix(possNewQuats(:,i));
        possExpMagMeas(1:3,i) = Aq*magField;
    end
end