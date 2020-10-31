function yK1 = predictMeas2(lambda,gammaK1,n)
    %
    % predictMeas following research paper instructions closely
    predictionSum = lambda*gammaK1(:,2*n+1);
    
    for i = 1:(2*n)
        predictionSum = predictionSum + gammaK1(:,i)/2;
    end
    
    yK1 = predictionSum/(n+lambda);
end