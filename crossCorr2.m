function PK1xy = crossCorr2(ChiK1,meanMinus,gammaK1,yK1,lambda,n)
    %
    % following paper exactly
    CovSum = lambda*(ChiK1(:,2*n+1) - meanMinus)*transpose(...
        gammaK1(:,2*n+1) - yK1);
    
    for i = 1:(2*n)
        CovSum = CovSum + (ChiK1(:,i) - meanMinus)*transpose(...
            gammaK1(:,i) - yK1)/2;
    end
    
    PK1xy = CovSum/(n+lambda);
end