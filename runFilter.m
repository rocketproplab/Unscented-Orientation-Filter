function [v2errQs,v1errQs] = runFilter(attitudeQuat,covariance,gyrobias)
    %% Information
    %
    % runFilter(attitudeQuat,covariance,gyrobias)
    % by Matthew Mikhailov, based on Crassidis and Markley
    %
    % Initializes the Kalman filter
    % attitudeQuat is the initial attitude quaternion
    %   - four vector with cos term 4th
    % covariance is a 6x6 matrix
    %   - it has attitude error elements and bias error elements. I believe
    %       is a diagonal matrix
    %   - figure out how to find initial attitude and covariance. Try to 
    %       find a math relationship between sd and cov.
    % gyrobias is a three vector gyro bias estimate currently in deg/h
    %
    %
    % This is an unscented filter for spacecraft attitude estimation, which
    %   keeps track of attitude with a quaternion parameterization, and 
    %   keeps track of attitude error using a generalized three vector 
    %   parameterization. The authors Crassidis and Markley call it the 
    %   unscented quaternion estimator (USQUE). They tested it using 10 
    %   second measurement periods over the course of hours; we plan on 
    %   using it with ~100Hz measurements over a few minutes.
    % See description.txt for more information.
    
   
    % TODO: Check units, in particular time units, for all constants and
    %   functions.
    
    
    %% Simulation Parameters
    
    
    % TODO: figure out how to find standard deviation values. 
    % See description.txt for details.
    
    
    % rotationTime is the time it takes for the spacecraft to complete a
    %   single rotation in seconds.
    % A full rotation every 90 minutes
    rotationTime = 5400;
    
    
    % rotVec is the vector around which the spacecraft rotates in the
    %   simulation
    rotVec = [1;2;3];
    
    
    % runTime is the length of our filter simulation in seconds.
    % On the spacecraft, the filter can run continuously.
    runTime = 32400;
    
    
    % gyroDt is the gyro and mag sampling interval in s.
    gyroDt = 10;
    
    
    % estimated gyro bias standard deviation in rad per s^(3/2).
    sigma_u = 3.1623e-10;
    
    
    % estimated gyro noise standard deviation in rad per s^(1/2).
    sigma_v = 0.31623e-6;
    
    
    % estimated magnetometer noise standard deviation in Tesla.
    mag_sd = 50e-9;
    
    
    %% Constants and Fine-Tuning
    
    
    % Parameters for fine-tuning filter.
    % Try (a,lambda) values (1,1), (1,0), (1,-1), and (2,-1)
    a = 1;          % a is a gRp parameter 0<=a<=1
    lambda = 1;     % lambda is a sigma point generation parameter
    f = 2*(a+1);    % f is a gRp parameter
    
    
    % Half the number of sigma points (half are positive, half negative).
    % This value should never change.
    n = 6;
    
    
    %% Initial Calculations
    
    
    % The number of filter iterations for this simulation.
    iterations = round(runTime/gyroDt);
    
    
    % initialize state vector (the postupdate mean error)
    meanPlus = [0;0;0;gyrobias];   
    
    
    % Eq. (42)
    % QbarK is only based on the gyro bias and noise properties and
    %   thus does not change throughout the operation of the filter
    QbarK = processNoise(gyroDt,sigma_v,sigma_u);
     
    % Declare arrays for gyroMeas and magMeas, to record the values for a
    % simulation, so the simulation can be rerun.
    v1 = attitudeQuat;
    gyroVals = zeros(3,iterations);
    magVals = zeros(3,iterations);
    idealGyroVals = zeros(3,iterations);
    v2errQs = zeros(4,iterations);
    v1errQs = zeros(4,iterations);
    
    %% Filter Loop
    % The code above gets the initial attitudeQuat and covariance, but 
    %   after that the attitudeQuat and covariance for each loop would be 
    %   found inside the previous iteration.
    for i=1:iterations
        
        %% Test Simulation
        
        % Generate the actual angular velocity of the rocket. Currently,
        %   always returns the same value, but you can make it dependent on
        %   i, the iteration number and thus point in time.
        [idealAngV,currentQuat] = idealPath(rotationTime,rotVec,gyroDt);
        idealGyroVals(:,i) = idealAngV;
        
        % TODO: Get gyroMeas
        gyroMeas = readGyro(idealAngV);
        gyroVals(:,i) = gyroMeas;
        % TODO: get WMM total intensity vector F
        F = readWMM();
        % TODO: Get magMeas, the measurement from the magnetometer
        % TODO: consider normalizing F for all comparisons
        magMeas = readMag(currentQuat,F);
        magVals(:,i) = magMeas;
        
        % Get orientation tracking version 1 results
        v1 = gyroIntegrate(v1,gyroMeas,gyroDt);
        
        %% Propagation Step
        % Eq.s (5),(33),(32)
        % Decompose the postupdate (most recent) covariance and QbarK (a
        %   process noise related covariance), to get vectors that 
        %   represent the error distribution. Add the expected error 
        %   (updated mean state) to them and transform them into quaternion
        %   errors, and then into quaternions. The result is 13 quaternions
        %   (6 with positive errors, followed by 6 with negative errors and
        %   one with "no errors" at the end of the array) that represent 
        %   the distribution of possible attitude quaternions. Also return 
        %   the bias components of the generated chi values, since these 
        %   components will not change between propagations.
        [qK,chiBias] = sigmaQuats(lambda,covariance,QbarK,meanPlus,a,f,...
        attitudeQuat,n);
        
    
        % In paper, readGyro() occurs here
        
        
        % Eq. (35)
        % Create a distribution of possible angular velocities, 
        %   corresponding to different possible gyro bias values.
        omegaK = sigmaOmegas(gyroMeas,meanPlus,chiBias,n);
        
        % TODO: Consider storing omegaK(0), since this is the best estimate
        %   of the rocket's angular velocity.
        
        
        % Eq. (34)
        % Propogate the 13 qK sigma quaternions using the 13 angular 
        %   velocities to get 13 qK1 propagated sigma quaternions.
        qK1 = quatPropagate(qK,omegaK,gyroDt);
        
        
        % Eq.s (36), (37), and (38)
        % Find the propagated error quaternions. From them, find the 
        %   attitude error part of the propagated Chi values. Add chiBias 
        %   values to the bottom to get propagated Chi values.
        ChiK1 = newChis(qK1,chiBias,n,f,a);
        
        
        %% Prediction Step
        % Eq. (7) and (8)
        % Now that we have our propagated Chi values  we can find our 
        %   predicted mean error (pre-update), and our predicted covariance
        %   (pre-update).
        [meanMinus,PminusK1] = predictError(lambda,ChiK1,QbarK,n);
        
        
        % Magentic Data
        % Use the International Goemagnetic Reference Field or WMM to find
        %   the magnetic field at the rocket's current location. Input 
        %   latitude, longitude, altitude, date. 
        % Magnetometer most likely senses total intensity, so compare 
        %   with total intensity. Consider normalizing expectation and 
        %   measurement vectors, and consider removing other data entries 
        %   from data sheet for a smaller sheet.
        
        % In paper, readWMM() occurs here
        
        
        % Eq. (43)
        % For each sigma quaternion, generate an expected measurement
        % If you're adding a measurement, you can add it inside this
        %   function.
        gammaK1 = sigmaMeas(qK1,F,n);
        
        
        % Eq. (9)
        % Find the mean expected observation vector
        yK1 = predictMeas(lambda,gammaK1,n);
        
        
        % Eq. (11), (12), and (23b)
        % Find the output covariance and measurement error noise
        %   covariance, using the expected measurements and magnetometer
        %   variance, respectively. Use these to find the innovation
        %   covariance.
        PK1vv = innovationCov2(gammaK1,yK1,lambda,mag_sd,n);
        
        
        % Eq. (13)
        % Find the cross-correlation matrix
        PK1xy = crossCorr2(ChiK1,meanMinus,gammaK1,yK1,lambda,n);
        
        
        %% Update Step
        % In paper, readMag() occurs here
        
        
        % Eq. (3), (4), (2a) and (2b)
        % Find the innovation and gain, and use them to find the updated, 
        %   propagated state (mean error vector) and the updated, 
        %   propagated covariance
        % These are important results of each loop, and will be
        %   used in the next loop.
        [meanPlus,covariance] = updateError(magMeas,yK1,PK1xy,PK1vv,...
            meanMinus,PminusK1);
        
        %% Final Quaternion, Recording, and Resetting
        % Eq. (45a), (45b), and (44)
        % Find the quaternion update, and use it to update the propogated
        %   attitude quaternion. This gives you the final attitude
        %   quaternion for this iteration.
        % This is one of the important results of each loop, and will be
        %   used in the next loop.
        attitudeQuat = quatUpdate(meanPlus,f,a,qK1,n);
        
        % Not technically part of the filter, used for testing purposes.
        % Find an error quaternion, which, if multiplied by attitudeQuat on
        %   the right, returns the actual ideal orientation quaternion of
        %   the rocket. Store this for each loop iteration.
        v2errQs(:,i) = kalmanArrayMult(currentQuat,kalmanArrayInv(...
            attitudeQuat));
        % orientation version 1 errors
        v1errQs(:,i) = kalmanArrayMult(currentQuat,kalmanArrayInv(v1));
        
        % Instruction immediately after Eq. (45b)
        % Reset the first three components of meanPlus to zero 
        %   for the next propagation.
        meanPlus = resetMean(meanPlus);
    end
    
    % Save vector and quaternion arrays to xls file
%     writematrix(gyroVals,append(char(datetime('now')),'_gyro','.csv'));
%     writematrix(magVals,append(char(datetime('now')),'_mag','.csv'));
%     writematrix(idealGyroVals,append(char(datetime('now')),'_ideal',...
%         '.csv'));
%     writematrix(errorQuats,append(char(datetime('now')),'_errorQuats',...
%         '.csv'));
end

    
    
    
        
    
    
        
    
    
    
    
    
    
    
    
    
    