function gammaK1 = sigmaMeas(qK1,F,n)
    % Eq. (43)
    % For each sigma quaternion, generate an expected measurement
    % If you're adding a measurement, and you have an expected vector
    %   (in the world frame) for the measurement, you can add it here.
    %   Just add another three components to the bottom of each gammaK1
    %   column vector, and update the size you are initializing the 
	%   gammaK1 array to.
	gammaK1 = zeros(3,2*n+1);
    for i=1:(2*n+1)
        Aq = attitudeMatrix(qK1(:,i));
        gammaK1(1:3,i) = Aq*F;
    end
end