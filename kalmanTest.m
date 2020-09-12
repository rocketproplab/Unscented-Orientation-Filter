function tests = kalmanTest
tests = functiontests(localfunctions);
end

function testProcessNoise(testCase)
act = processNoise(5,7,3);
exp = (diag([11.5;11.5;11.5;9;9;9]))*(5/2);
verifyEqual(testCase,act,exp)
end

function testCrossMatrix(testCase)
act = crossMatrix([1;2;3]);
exp = [0,-3,2;3,0,-1;-2,1,0];
verifyEqual(testCase,act,exp)
end

function testAttitudeMatrix(testCase)
act = attitudeMatrix([1;2;3;4]/(sqrt(30)));
exp = (1/30)*[4,28,-10;-20,10,20;22,4,20];
verifyEqual(testCase,act,exp,'RelTol',1e-12)
end

function testArrayInv(testCase)
act = kalmanArrayInv([1;2;3;4]/(sqrt(30)));
exp = [-1;-2;-3;4]/(sqrt(30));
verifyEqual(testCase,act,exp)
end

function testArrayMult(testCase)
act = kalmanArrayMult([1;2;3;4]/(sqrt(30)),[5;7;11;13]/(sqrt(364)));
exp = [34;58;80;0]/(sqrt(10920));
verifyEqual(testCase,act,exp)
end



function testSigmaQuats(testCase)

end

function testSigmaOmegas(testCase)
act = sigmaOmegas(repmat(6,3,13),[0;0;0;1;2;3],repmat([1:13],3,1),6);
exp = [repmat(5:-6,3,1), [6;6;6;5;4;3]];
verifyEqual(testCase,act,exp)
end

function testPropagate(testCase)
end

function testNewChis(testCase)
end

function testPredictError(testCase)
end

function testPredictMeas(testCase)
end

function testInnovCov(testCase)
end

function testcrossCorr(testCase)
end

function testUpdateError(testCase)
end

function testQuatUpdate(testCase)
end

function testResetMean(testCase)
end

