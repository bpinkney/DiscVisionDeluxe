% Gaussian Noise Generator
% generate numbers with mean 0 and standard deviation 1. 
% (To adjust to some other distribution, multiply by the standard deviation and add the mean.)
% ~0.65s for 10000000 rands on X86
function out = gaussrand(V1, V2, S, phase)

  X = 0;
  
  if(phase == 0)
    while(S >= 1 || S == 0)
      U1 = double(rand);% / (float)RAND_MAX;
      U2 = double(rand);% / (float)RAND_MAX;

      V1 = 2 * U1 - 1;
      V2 = 2 * U2 - 1;
      S = V1 * V1 + V2 * V2;
    end
    X = V1 * sqrt(-2 * log(S) / S);
  else
    X = V2 * sqrt(-2 * log(S) / S);
  end

  phase = 1 - phase;
  
  out = X;
end