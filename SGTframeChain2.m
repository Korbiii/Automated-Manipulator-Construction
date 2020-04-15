function [LFrm]=SGTframeChain2(nums,varargin)
split = ((nums-1)/2)+2; if nargin>=2 && ~isempty(varargin{1}); split=varargin{1}; end

SGt = cell(nums,5);
for i=1:nums
    SGt{i,1} = i;
    SGt{i,2} = 'F';
    SGt{i,3} = 'B';
    SGt{i,4} = i-1;
    SGt{i,5} = i;
end
SGt{1,2} = '_';
current = 2;
for i = 1: size(split,2)
    current = current +split(i);
    SGt(current,2) ={['F' num2str(i)]};
    SGt(current,4) = {[1]};
end

LFrm = SGt;
end