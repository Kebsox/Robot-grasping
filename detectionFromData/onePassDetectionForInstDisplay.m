% Helper function which reads out the BG file name from a list by instance
% 
% Author: Ian Lenz

function [bestRect,bestScore] = onePassDetectionForInstDisplay(dataDir,bgDir,bgNos,instNum,w1,w2,wClass,means,stds,rotAngs,hts,wds,scanStep,modes)

bgFN = sprintf('%s/pcdb0002r.png',bgDir);

[bestRect,bestScore] = onePassDetectionNormDisplay(dataDir,bgFN,instNum,w1,w2,wClass,means,stds,rotAngs,hts,wds,scanStep,modes);