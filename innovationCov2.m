function PK1vv = innovationCov2(gammaK1,yK1,lambda,mag_sd,n)
    %
    % Stricltly following research paper
    CovSum = lambda*(gammaK1(:,2*n+1) - yK1)*transpose(gammaK1(:,2*n+1)...
        - yK1);
    
    for i = 1:(2*n)
        CovSum = CovSum + (gammaK1(:,i) - yK1)*transpose(gammaK1(:,i) - ...
            yK1)/2;
    end
    
    PK1yy = CovSum/(n+lambda);
    
    PK1vv = PK1yy + (mag_sd^2)*eye(3);
end