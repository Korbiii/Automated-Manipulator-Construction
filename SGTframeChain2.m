function [LFrm]=SGTframeChain2(nums,varargin)
split = ((nums-1)/2)+2; if nargin>=2 && ~isempty(varargin{1}); split=varargin{1}+2; end

SGt = cell(nums,5);
for i=1:nums
    SGt{i,1} = i;
    SGt{i,2} = 'F';
    SGt{i,3} = 'B';
    SGt{i,4} = i-1;
    SGt{i,5} = i;
end
SGt{1,2} = '_';

SGt(split,2) ={'F1'};
SGt(split,4) = {[1]};

LFrm = SGt;
end