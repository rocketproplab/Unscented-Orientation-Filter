function [meanMinus,PminusK1] = predictError2(lambda,ChiK1,QbarK,n)
    % predictError2 attempts the same task as predictError, but it does not
    %   simplify the research paper math to a matrix product, and instead
    %   attempts to replicate the paper directly.
    ChiKSum = lambda*ChiK1(:,2*n+1);
    for i = 1:(2*n)
        ChiKSum = ChiKSum + ChiK1(:,i)/2;
    end
    meanMinus = ChiKSum/(n+lambda);
    
    CovarSum = lambda*(ChiK1(:,2*n+1) - meanMinus)*transpose(...
        ChiK1(:,2*n+1) - meanMinus);
    
    for i = 1:(2*n)
        CovarSum = CovarSum + (ChiK1(:,i) - meanMinus)*transpose(...
            ChiK1(:,i) - meanMinus)/2;
    end
    PminusK1 = CovarSum/(n+lambda) + QbarK;
end