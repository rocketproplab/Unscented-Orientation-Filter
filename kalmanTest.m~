function tests = kalmanTest
tests = functiontests(localfunctions);
end



function testProcessNoise(testCase)
act = processNoise(5,7,3);
exp = (diag([11.5;11.5;11.5;9;9;9]))*(5/2);
verifyEqual(testCase,act,exp)
end

function testRotQuat(testCase)
act = rotQuat(pi/3,([1,2,3]/(sqrt(14))));
exp = ([1;2;3;0]/(2*sqrt(14))) + ([0;0;0;1]*(sqrt(3)/2));
verifyEqual(testCase,act,exp,'RelTol',1e-12)
end

function testMultQuat(testCase)
act = multQuat([2;3;5;7],[11;13;17;19]);
exp = [101;169;207;-13];
verifyEqual(testCase,act,exp)
end

function testGyroIntegrate(testCase)
quatTest2()
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
a = [2;3;5;7];
b = kalmanArrayInv(a);
verifyEqual(testCase,multQuat(a,b),[0;0;0;1])
end

function testArrayMult(testCase)
act = kalmanArrayMult([2;3;5;7],[11;13;17;19]);
exp = [101;169;207;-13];
verifyEqual(testCase,act,exp)
end

function testArrayMult2(testCase)
act = kalmanArrayMult2([2;3;5;7],[11;13;17;19]);
exp = [101;169;207;-13];
verifyEqual(testCase,act,exp)
end





function testIdealPath(testCase)
clear idealPath;
idealAngV = [0;0;0];
currentQuat = [0;0;0;0];
    for i=1:540
        [idealAngV,currentQuat] = idealPath(5400,[1;1;1],10);
    end
verifyEqual(testCase,idealAngV,(pi/(2700*sqrt(3)))*[1;1;1],'RelTol',1e-12);
verifyEqual(testCase,abs(currentQuat),[0;0;0;1],'AbsTol',1e-12)
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


