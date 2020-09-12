function omegaK = sigmaOmegas(gyroMeas,meanPlus,chiBias,n)
    

    % Eq. (35)
    % Create a sigma estimated angular velocity array using
    %   Chi bias sigma vectors
    omegaK = zeros(3,2*n+1);
    for i=1:(2*n)
        omegaK(:,i) = gyroMeas-chiBias(:,i);
    end
    
    % Add the omegaK(0) three vector to the end of the array. The omegaK
    %   array now has 13 columns.
    omegaK(:,2*n+1) = gyroMeas - meanPlus(4:6,1);
end