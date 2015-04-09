classdef Img
properties(Constant)
DefaultChainTable=[0 1 1  1  0 -1 -1 -1;
                   1 1 0 -1 -1 -1  0 1];
end

methods(Static)

function A=set_edge(A,val,width)
if ~exist('val','var'); val=0; end;
if ~exist('width','var'); width=1; end;
A(1:width,:)=val;
A((end+1-width):end,:)=val;
A(:,1:width)=val;
A(:,(end+1-width):end)=val;
end

function code=trace_clockwise(A,row,col,prev_code)
val=A(row,col);               
start_code=mod(prev_code+2,8);               
ChainTable=circshift(Img.DefaultChainTable,[0 -start_code]);
for k=1:length(ChainTable)
    if A(row+ChainTable(1,k),col+ChainTable(2,k)) == val
        code=mod(start_code+k-1,8);
        return
    end;
end
code=-1; % no neighbourhood
end    

function X=contour_trace(A,row,col)
CodeTransitionTable=[4 5 6 7 0 1 2 3];               
trace_clockwise_inner=@(r,c,prev) Img.trace_clockwise(A,r,c,prev);

S=[row;col];
X=S;
code=trace_clockwise_inner(row,col,5);
if code==-1; return; end;
Tcode=code;
U=X(:,end)+Img.DefaultChainTable(:,code+1);
X(:,end+1)=U;
while(true)
    code=trace_clockwise_inner(U(1,1),U(2,1),CodeTransitionTable(1,code+1));
    U=X(:,end)+Img.DefaultChainTable(:,code+1);
    if X(1,end)==S(1,1) && X(2,end)==S(2,1) && Tcode==code
        X=X(:,1:end-1);
        break;
    end
    X(:,end+1)=U;
end
end

function RowCol = label_start_points(L,N)
if ~exist('N','var'); N=max(L(:)); end;
RowCol=zeros(N,2);
for I=1:N
    [rows,cols]=find(L==I);
    RowCol(I,:)=[rows(1) cols(1)];
end
end

function Cs = contours(L,N)
% 0 or 1 img
if ~exist('N','var'); N=max(L(:)); end;
Cs=cell(N,1);
for I=1:N
    [rows,cols]=find(L==I);
    Cs{I}=Img.contour_trace(L,rows(1),cols(1))';
end
end

function Cs = contours_in_length(L,N,min_len,max_len)
% 0 or 1 img
if ~exist('N','var'); N=max(L(:)); end;
if ~exist('min_len','var'); min_len=0; end;
if ~exist('max_len','var'); max_len=numel(L); end;
Cs=cell(N,1);
J=1;
for I=1:N
    [rows,cols]=find(L==I);
    len=length(rows);
    if min_len <= len && len <= max_len
%             fprintf(1,'[contour size]%d\n',len);
        Cs{J}=Img.contour_trace(L,rows(1),cols(1))';
        J=J+1;
    end
end
Cs=Cs(1:(J-1),1);
end

function Cs = contours_slow(L,N)
% 0 or 1 img
if ~exist('N','var'); N=max(L(:)); end;
RowCol=Img.label_start_points(L,N);
Cs=cell(N,1);
for I=1:N
    Cs{I}=Img.contour_trace(L,RowCol(I,1),RowCol(I,2))';
end
end

function [lambdaS lambdaL]=corner_measure(RowCol,win)
% RowCol = [row1 col1; row2 col2; ... rown coln];
if ~exist('win','var'); win = size(RowCol,1)/4; end;
% k: Half of Window Size (Window = j-k j-k+1 ... j-1 j j+1 ... j+k = 2k+1)
k=floor(win/2);
X=RowCol(:,1);
Y=RowCol(:,2);
X2=X.^2;
Y2=Y.^2;
XY=X.*Y;
n=numel(X); % assert((2*k+1)<(n/2+1)); % window size ÇÕ n/2 à»â∫Ç≈Ç†ÇÈÇ±Ç∆
CX=zeros(n,1);
CY=zeros(n,1);
CX2=zeros(n,1);
CY2=zeros(n,1);
CXY=zeros(n,1);
for s=1:n
    if s-k < 1 
        w=[(n+(s-k)):n 1:(s+k)];
    elseif s+k > n
        w=[(s-k):n 1:(s+k-n)];
    else
        w=(s-k):(s+k);
    end
    CX(s)=sum(X(w));
    CY(s)=sum(Y(w));
    CX2(s)=sum(X2(w));
    CY2(s)=sum(Y2(w));
    CXY(s)=sum(XY(w));
end
CX=CX./(2*k+1);
CY=CY./(2*k+1);
C11=CX2./(2*k+1)-CX.^2;
C22=CY2./(2*k+1)-CY.^2;
C12=CXY./(2*k+1)-CX.*CY;
lambdaS=(C11+C22-sqrt((C11-C22).^2+4*C12.^2))/2.0;
lambdaL=(C11+C22+sqrt((C11-C22).^2+4*C12.^2))/2.0;
end

function Cs=corner_detect(RowCol, win, k, h)
% Row = [row1 col1; row2 col2; ... rown coln];
assert(size(RowCol,2)==2);
if ~exist('win','var'); win = size(RowCol,1)/4; end;
if ~exist('k','var'); k=5; end; % k: half window size (3Å`5)
if ~exist('h','var'); h=1; end; % h: threshold (1Å`3)
Ls=Img.corner_measure(RowCol,win);
[~,I]=Util.peak_detect(Ls,k,h);
Cs=RowCol(I,:);
end

function Box=bounding_box_of_polygon(RowCol)
Box=[min(RowCol(:,1));min(RowCol(:,2));max(RowCol(:,1));max(RowCol(:,2))];
end

function isContains=bounding_box_contains(RowCol1, RowCol2)
B1=Img.bounding_box_of_polygon(RowCol1);
B2=Img.bounding_box_of_polygon(RowCol2);
isContains= B1(1) <= B2(1) && B1(2) <= B2(2) && B2(3) <= B1(3) && B2(4) <= B1(4);
end

function Fs=filter_outers(Cs)
N=length(Cs);
IDX=1:N;
for I=1:length(Cs)
    C=Cs{I};
    for J=1:length(Cs)
        D=Cs{J};
        if J~=I && Img.bounding_box_contains(C,D)
            IDX(I)=0;
            break;
        end
    end
end
IDX=find(IDX ~= 0);
Fs=Cs(IDX);
end

function A=draw_points(A,ps,val)
for I=1:size(ps,1)
    A(ps(I,1),ps(I,2))=val;
end
end

function A=draw_points_list(A,ps_list,val)
for I=1:size(ps_list,1)
    ps=ps_list{I};
    for J=1:size(ps,1)
        A(ps(J,1),ps(J,2))=val;
    end
end
end

function A=get_marker_red_bin(I,t)
if ~exist('t','var'); t=90; end;  
R=I(:,:,1);
G=I(:,:,2);
B=I(:,:,3);
A=(R-max(G,B))>t;
end

function A=get_marker_green_bin(I,t)
if ~exist('t','var'); t=20; end;
R=I(:,:,1);
G=I(:,:,2);
B=I(:,:,3);
A=(G-max(B,R))>t;
end

function A=get_marker_blue_bin(I,t)
if ~exist('t','var'); t=60; end;   
R=I(:,:,1);
G=I(:,:,2);
B=I(:,:,3);
A=(B-max(R,G))>t;
end


function A=get_marker_cyan_bin(I,t)
if ~exist('t','var'); t=80; end;   
R=I(:,:,1);
G=I(:,:,2);
B=I(:,:,3);
A=(min(G,B)-R)>t;
end

function A=get_marker_magenta_bin(I,t)
if ~exist('t','var'); t=25; end;   
R=I(:,:,1);
G=I(:,:,2);
B=I(:,:,3);
A=(min(B,R)-G)>t;
end

function A=get_marker_yellow_bin(I,t)
if ~exist('t','var'); t=130; end;   
R=I(:,:,1);
G=I(:,:,2);
B=I(:,:,3);
A=(min(R,G)-B)>t;
end

function A=get_marker_red_bin_hsv(H,S,V,th,ts)
if ~exist('th','var'); th=0.05; end;  
if ~exist('ts','var'); ts=0.85; end;  
A=(abs(H-0)<th & S>ts);
end

function A=get_marker_green_bin_hsv(H,S,V,th,ts)
if ~exist('th','var'); th=0.08; end;  
if ~exist('ts','var'); ts=0.85; end;  
A=(abs(H-2/6)<th & S>ts);
end

function A=get_marker_blue_bin_hsv(H,S,V,th,ts)
if ~exist('th','var'); th=0.11; end;  
if ~exist('ts','var'); ts=0.65; end;  
A=(abs(H-4/6)<th & S>ts);
end

function A=get_marker_cyan_bin_hsv(H,S,V,th,ts)
if ~exist('th','var'); th=0.035; end;  
if ~exist('ts','var'); ts=0.65; end;  
A=(abs(H-3/6-0.01)<th & S>ts);
end

function A=get_marker_yellow_bin_hsv(H,S,V,th,ts)
if ~exist('th','var'); th=0.035; end;  
if ~exist('ts','var'); ts=0.75; end;  
A=(abs(H-1/6)<th & S>ts);
end

function X=centroid_bin(I)
[rows,cols]=find(I>0);
X=[mean(cols);mean(rows)];
end

function b=bounds_contain(siz,X,edge_width)
if ~exist('edge_width','var'); edge_width=0; end;
b=(prod(double(X(:) > edge_width)) * prod(double([X(1,:)-siz(2)+edge_width X(2,:)-siz(1)+edge_width]<=0)))>0;
end

end
end
