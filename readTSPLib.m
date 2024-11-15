function p=readTSPLib(file_name, varargin)
% read file from TSPlib and return a sequence of points to visit

% p: locations of points of interest (2xN matrix)
%   points are randomly extracted for the table of points in the input file
%   p(:,1)=p(:,N) is assumed to be both the initial and final point

% file_name is a string with the name of the input file to read
% the file must be a .txt file
% the file must contain a table with 2D coordinates
% the file can follow the format of TSPLib instances, overhead information is automatically removed
% the file can be obtained by chaning the file extension of .tsp files from TSPLib to .txt

% varargin{1}=n desired number of points. 
%   if empty, all the points contained in the file are used
% varargin{2}=w desired size of the area.
%   if not empty, point locations are scaled to lay in a WxW area



P=readtable(file_name); 
points=table2array(P)';
points=points(2:3,1:end-1); % get vector with all points locations in the input file
N0=size(points,2);
x0=min(points(1,:));
y0=min(points(2,:));
Wx=max(points(1,:))-min(points(1,:));
Wy=max(points(2,:))-min(points(2,:));
W0=max([Wx,Wy]);

if length(varargin)==0
    N=N0;
    W=W0;
elseif length(varargin)==1
    N=varargin{1};
    W=W0;
else
    N=varargin{1};
    W=varargin{2};
end

index_perm=randperm(N0);
index_selected=index_perm(1:N); 
p=points(:, index_selected); % extract N random points 
p=[p p(:,1)]; % p(:,1) is assumed to be both the initial and the final point
p=p-[x0;y0]; % shift point locations towards the origin of the cartesian space
p=W/W0*p; % scale point locations to lay in a WxW area