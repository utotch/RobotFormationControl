classdef Marker
properties(Constant)
A_15_11=[...
Marker.int2bitvec4(3),...    
Marker.int2bitvec4(5),...    
Marker.int2bitvec4(6),...    
Marker.int2bitvec4(7),...    
Marker.int2bitvec4(9),...    
Marker.int2bitvec4(10),...    
Marker.int2bitvec4(11),...    
Marker.int2bitvec4(12),...    
Marker.int2bitvec4(13),...    
Marker.int2bitvec4(14),...    
Marker.int2bitvec4(15)];
H_15_11=[Marker.A_15_11 eye(4,4)]; % Hamming Code Check Matrix
H_15_11_int2idx=[15 14 1 13 2 3 4 12 5 6 7 8 9 10 11];
G_15_11=[eye(11,11) Marker.A_15_11']; % Hamming Code Generator Matrix
H_16_12ex=[Marker.A_15_11 eye(4,4) zeros(4,1); ones(1,16)];
% G_16_12ex=[logical(eye(11,11)) Marker.A_'];
end
methods(Static)
    
function X=int2bitvec4(a)
X=[bitand(bitshift(a,-3),1); bitand(bitshift(a,-2),1); bitand(bitshift(a,-1),1); bitand(a,1)];
end

function X=int2bitvec11(a)
X=[...
   bitand(bitshift(a,-10),1);...
   bitand(bitshift(a,-9),1);...
   bitand(bitshift(a,-8),1);...   
   bitand(bitshift(a,-7),1);...    
   bitand(bitshift(a,-6),1);...    
   bitand(bitshift(a,-5),1);...    
   bitand(bitshift(a,-4),1);...    
   bitand(bitshift(a,-3),1);...
   bitand(bitshift(a,-2),1);...
   bitand(bitshift(a,-1),1);...
   bitand(a,1)];
end

function I=bitvec4_to_int(v)
I=int32(v(1)*8 + v(2)*4 + v(3)*2 + v(4));
end

function I=bitvec11_to_int(v)
I=int32(v(1)*1024 + v(2)*512 + v(3)*256 + v(4)*128 + v(5)*64 + v(6)*32 ...
    + v(7)*16 + v(8)*8 + v(9)*4 + v(10)*2 + v(11));
end

function X=calc_calibration_points(I)
DefaultOpenSize=3;    
X=zeros(2,4);
funs={@(x) Img.get_marker_red_bin(x),...
      @(x) Img.get_marker_green_bin(x),...      
      @(x) Img.get_marker_blue_bin(x),...
      @(x) Img.get_marker_cyan_bin(x)};
for J=1:length(funs)
    fun=funs{J};
    A=fun(I);
    B=imopen(A,strel('square',DefaultOpenSize));
    X(:,J)=Img.centroid_bin(B);
    if isnan(X(1,J))
        throw(MException('CalibrationError:ColorMarkerNotFound',...
            'CalibrationByColor:%s fail',func2str(fun)));
    end
end
end

function X=calc_calibration_points_hsv(H,S,V)
DefaultOpenSize=3;    
X=zeros(2,4);
funs={@(h,s,v) Img.get_marker_red_bin_hsv(h,s,v),...
      @(h,s,v) Img.get_marker_green_bin_hsv(h,s,v),...      
      @(h,s,v) Img.get_marker_blue_bin_hsv(h,s,v),...
      @(h,s,v) Img.get_marker_cyan_bin_hsv(h,s,v)};
for J=1:length(funs)
    fun=funs{J};
    A=fun(H,S,V);
    B=imopen(A,strel('square',DefaultOpenSize));
    X(:,J)=Img.centroid_bin(B);
    if isnan(X(1,J))
        throw(MException('CalibrationError:ColorMarkerNotFound',...
            'CalibrationByColor:%s fail',func2str(fun)));
    end
end
end

function [H,invH]=calc_homography_matrix(X)
assert(size(X,1)==2 && size(X,2)==4);
Xc=[X; ones(1,4)];
Xn=[-1 -1  1  1;
     1 -1 -1  1;
     1  1  1  1]; 
N=size(Xc,2);
A=zeros(2*N,8); 
b=zeros(2*N,1);
for I=1:N
xi=Xc(1,I); yi=Xc(2,I); wi=Xc(3,I);
xpi=Xn(1,I); ypi=Xn(2,I); wpi=Xn(3,I);
rows=2*I:(2*I+1);
A(rows,:)=...
[0      0      0      -xi*wpi -yi*wpi -wi*wpi  xi*ypi  yi*ypi;
 xi*wpi yi*wpi wi*wpi 0       0       0       -xi*xpi -yi*xpi];
b(rows,:)=[-wi*ypi; wi*xpi];
end
h=A \ b;
H=reshape([h;1],3,3)';
invH=inv(H);
end

function X=template_points(corners)
% (1)-(4)
%  |   |
% (2)-(3)
% corners=[x1 x2 x3 x4; y1 y2 y3 y4]
assert(size(corners,1)==2 && size(corners,2)==4);
[H,invH]=Marker.calc_homography_matrix(corners);
Xn=[-3/8 -3/8 -3/8 -3/8 -1/8 -1/8 -1/8 -1/8  1/8  1/8  1/8   1/8 3/8  3/8  3/8  3/8;
     3/8  1/8 -1/8 -3/8  3/8  1/8 -1/8 -3/8  3/8  1/8 -1/8  -3/8 3/8  1/8 -1/8 -3/8;
    ones(1,16)];
% y方向は、下向きが＋になっているため、サンプリング方向が逆になる？？
% Xn=[-3/8 -3/8 -3/8 -3/8 -1/8 -1/8 -1/8 -1/8   1/8  1/8  1/8  1/8   3/8  3/8  3/8  3/8;
%     -3/8 -1/8  1/8  3/8 -3/8 -1/8  1/8  3/8  -3/8 -1/8  1/8  3/8  -3/8 -1/8  1/8  3/8;
%     ones(1,16)];
X=invH*Xn;
X=[X(1,:)./X(3,:); X(2,:)./X(3,:)];
end

function T=extract_template_simple(X,points)
assert(size(points,1)==2 && size(points,2)==16);
Cols=round(points(1,:));
Rows=round(points(2,:));
idx=sub2ind(size(X),Rows,Cols);
T=X(idx);
end

function T=extract_template_near(X,points)
assert(size(points,1)==2 && size(points,2)==16);
T=zeros(1,size(points,2));
for I=1:length(T)
    col=round(points(1,I));
    row=round(points(2,I));
    T(I)=mean([X(row,col) X(row-1,col) X(row+1,col) X(row,col-1) X(row,col+1)]) > 0.5;
end
end

function T=extract_template_mean(X,points,w)
assert(size(points,1)==2 && size(points,2)==16);
if ~exist('w', 'var'); w=1; end;
T=zeros(1,size(points,2));
for I=1:length(T)
    col=round(points(1,I));
    row=round(points(2,I));
    S=X((row-w):(row+w),(col-w):(col+w));
    T(I)=mean(S(:)) > 0.5;
end
end

function M=generate_markers1
numMarkers=10;    
M=zeros(4,4,numMarkers); 
figure(1);
for I=1:numMarkers
    M(:,:,I)=reshape([Marker.int2bitvec11(2^I)' * Marker.G_15_11 0],4,4);
    subplot(3,4,I);
    imshow(~M(:,:,I));
end

end

function [id,rot]=decode_template1_with_rotation(T)
% rot:何回rotation すると一致するか？    
id=-1;
for rot=0:3
if T(end)==0
    id=Marker.decode_template1(T);
    if id > 0 || rot==3
        break;
    end;
end
T=rot90(reshape(T,4,4));
T=T(:)';
end
end

function id=decode_template1(T)
assert(size(T,1)==1 && size(T,2)==16);
MaxID=9; % dirty code. Code(10) is same as rotation270(Code(3)). so ignore Code(10)
% id=-1;
YHt=bitand(T(1:15)*Marker.H_15_11',1);
if (sum(YHt)~=0) % err correction
    idx=Marker.H_15_11_int2idx(Marker.bitvec4_to_int(YHt));
    T(idx)=~T(idx);
end
I=Marker.bitvec11_to_int(T(1:11));
id=nextpow2(double(I));
if (I~=2^id || id > MaxID) % dirty code
   id=-1;
end
end

end
end
