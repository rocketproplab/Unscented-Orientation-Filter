function [meanMinus,PminusK1] = predictError(lambda,ChiK1,QbarK,n)
    % Now that we have our propagated Chi values  we can find our 
    %   predicted values of interest. Eq. (7) is for predicted mean
    %   error (pre-update).
    meanMinus = (lambda*ChiK1(:,2*n+1) ...
        + 0.5*sum(ChiK1(:,1:(2*n)),2))/(n+lambda);
        
    % Eq. (8)
    % Find the predicted covariance (pre-update).
    P_in = ChiK1(:,1:(2*n+1)) - repmat(meanMinus,1,2*n+1);
        
    PminusK1 = (lambda*(P_in(:,2*n+1))* transpose(P_in(:,2*n+1)) ...
        + 0.5*P_in(:,1:(2*n))*transpose(P_in(:,1:(2*n))))/(n+lambda) + ...
        QbarK;
end
        