function [LFrm]=SGTframeChain2(nums,varargin)


n = nums;
SGt = cell(n,5);
for i=1:n
    SGt{i,1} = i;
    SGt{i,2} = 'F';
    SGt{i,3} = 'B';
    SGt{i,4} = i-1;
    SGt{i,5} = i;
end
SGt{1,2} = '_';

sp = ((n-1)/2)+2;
SGt(sp,2) ={'F1'};
SGt(sp,4) = {[1]};

LFrm = SGt;
end