function test_suite=unit_test
    initTestSuite;
end

function test_trace_clockwise
assertEqual(Img.trace_clockwise([0 0 0; 0 3 3; 0 0 0],2,2,6),0);
func1=@(A,row,col) Img.trace_clockwise(A,row,col,6);
assertEqual(func1([0 0 0; 0 1 0; 0 0 0],2,2),-1);
assertEqual(func1([0 0 0; 0 1 1; 0 0 0],2,2),0);
assertEqual(func1([0 0 0; 0 1 0; 0 0 1],2,2),1);
assertEqual(func1([0 0 0; 0 1 0; 0 1 0],2,2),2);
assertEqual(func1([0 0 0; 0 1 0; 1 0 0],2,2),3);
assertEqual(func1([0 0 0; 1 1 0; 0 0 0],2,2),4);
assertEqual(func1([1 0 0; 0 1 0; 0 0 0],2,2),5);
assertEqual(func1([0 1 0; 0 1 0; 0 0 0],2,2),6);
assertEqual(func1([0 0 1; 0 1 0; 0 0 0],2,2),7);
assertEqual(func1([1 1 0; 1 1 0; 1 1 0],2,2),2);
assertEqual(Img.trace_clockwise([0 1 0; 0 1 0; 0 1 0],2,2,0),2);
assertEqual(Img.trace_clockwise([0 1 0; 0 1 0; 0 1 0],2,2,3),6);
assertEqual(Img.trace_clockwise([0 0 0 0; 0 1 1 0;0 0 0 0],2,3,4),4);
end

function test_contour_trace
assertEqual(Img.contour_trace([0 0 0; 0 1 0; 0 0 0],2,2),[2 2]');
assertEqual(Img.contour_trace([0 0 0 0; 0 1 1 0;0 0 0 0],2,2),[2 2; 2 3]');
assertEqual(Img.contour_trace([0 0 0 0; 0 1 1 0;0 1 1 0; 0 0 0 0],2,2),[2 2; 2 3; 3 3; 3 2]');
A=[0 0 0 0 0; 0 1 1 1 0; 0 1 0 1 0; 0 1 1 1 0; 0 0 0 0 0];
C=[2 2; 2 3; 2 4; 3 4; 4 4; 4 3; 4 2; 3 2]';
assertEqual(Img.contour_trace(A,2,2),C);
A=[0 0 0 0 0 0 0;
    0 0 0 1 0 0 0;
    0 0 1 0 1 1 0;
    0 1 0 0 0 0 0;
    0 0 0 0 0 0 0];
S=[2;4];T=[3;5];U=[3;6];V=[3;3];W=[4;2];
C=[S T U T S V W V];
assertEqual(Img.contour_trace(A,2,4),C);
end

function test_set_edge
A=ones(3);
B=[0 0 0; 0 1 0; 0 0 0];
C=[2 2 2; 2 1 2; 2 2 2];
assertEqual(Img.set_edge(A),B);
assertEqual(Img.set_edge(A,2),C);
end

function test_label_start_points
L=[0 0 0; 0 1 1; 0 0 2];
L2=[0 0 1; 0 1 1; 0 0 1];
assertEqual(Img.label_start_points(L,max(L(:))),[2 2; 3 3]);
assertEqual(Img.label_start_points(L),[2 2; 3 3]);
assertEqual(Img.label_start_points(L2),[2 2]);
end

function test_contours
L1=[0 0 0; 0 1 0; 0 0 0];
% L2=[0 0 0; 0 1 0; 0 2 2]; % ’[‚ªƒ[ƒ‚Å‚È‚¢—v‘f‚ª‚ ‚é‚Æƒ_ƒ
L2=[0 0 0 0; 0 1 0 0; 0 2 2 0; 0 0 0 0];
L3=[0 0 0 0; 0 1 1 0; 0 1 1 0; 0 0 0 0];
assertEqual(Img.contours(L1),{[2 2]});
assertEqual(Img.contours(L2),{[2 2]; [3 2; 3 3]});
assertEqual(Img.contours(L3),{[2 2; 2 3; 3 3; 3 2]});
end

function test_contours_in_length
L1=[0 0 0; 0 1 0; 0 0 0];
% L2=[0 0 0; 0 1 0; 0 2 2]; % ’[‚ªƒ[ƒ‚Å‚È‚¢—v‘f‚ª‚ ‚é‚Æƒ_ƒ
L2=[0 0 0 0; 0 1 0 0; 0 2 2 0; 0 0 0 0];
L3=[0 0 0 0; 0 1 1 0; 0 1 1 0; 0 0 0 0];

assertEqual(Img.contours_in_length(L1),{[2 2]});
assertEqual(Img.contours_in_length(L2),{[2 2]; [3 2; 3 3]});
assertEqual(Img.contours_in_length(L3),{[2 2; 2 3; 3 3; 3 2]});

assertEqual(Img.contours_in_length(L1,max(L1(:)),1,100),{[2 2]});
assertEqual(Img.contours_in_length(L2,max(L2(:)),2,100),{[3 2; 3 3]});
assertEqual(Img.contours_in_length(L2,max(L3(:)),0,1),{[2 2]});
assertEqual(Img.contours_in_length(L3,max(L3(:)),0,2),cell(0,1));
end

function test_draw_points
assertEqual(Img.draw_points(zeros(2),[1 1],5),[5 0;0 0]);
assertEqual(Img.draw_points(zeros(2),[1 1; 2 2],3),[3 0;0 3]);
end

function test_draw_points_list
assertEqual(Img.draw_points_list(zeros(2),{[1 1];[1 2]},5),[5 5;0 0]);
assertEqual(Img.draw_points_list(zeros(2),{[1 1];[2 2];[3 3]},1),[1 0 0;0 1 0; 0 0 1]);
end

function test_filter_cell
assertEqual(Util.filter_cell({1 2 3},@(x) x==2),{2});
assertEqual(Util.filter_cell({[1] [1 1] [1 1 1]},@(x) length(x)>=2),{[1 1] [1 1 1]});
assertEqual(Util.filter_cell({[1];[1 1];[1 1 1]},@(x) length(x)>=2),{[1 1];[1 1 1]});
end

function test_corner_measure
C0=[2 2; 2 3; 3 3; 3 2];
C1=[2 2; 2 3; 2 4; 3 4; 4 4; 4 3; 4 2; 3 2];
C2=[2 2; 2 3; 2 4; 2 5; 2 6; 2 7; 3 7; 4 7; 5 7; 6 7; 7 7; 7 6; 7 5; 7 4; 7 3; 7 2; 6 2; 5 2; 4 2; 3 2];
[Ls0 Ll0]=Img.corner_measure(C0,3);
[Ls1 Ll1]=Img.corner_measure(C1,3);
[Ls2 Ll2]=Img.corner_measure(C2,3);
assertEqual(size(Ls0),[size(C0,1) 1]);
assertEqual(size(Ls1),[size(C1,1) 1]);
assertEqual(size(Ls2),[size(C2,1) 1]);
assertEqual(prod(double(Ls0 <= Ll0)),1);
assertEqual(prod(double(Ls1 <= Ll1)),1);
assertEqual(prod(double(Ls2 <= Ll2)),1);

assertEqual(prod(double(Ls1(1) > Ls1([2 4 6 8]))),1);
assertEqual(prod(double(Ls1(3) > Ls1([2 4 6 8]))),1);
assertEqual(prod(double(Ls1(5) > Ls1([2 4 6 8]))),1);
assertEqual(prod(double(Ls1(7) > Ls1([2 4 6 8]))),1);

assertEqual(prod(double(Ls2(1) > Ls2([2 3 4 5  7 8 9 10  12 13 14 15  17 18 19 20]))),1);
assertEqual(prod(double(Ls2(6) > Ls2([2 3 4 5  7 8 9 10  12 13 14 15  17 18 19 20]))),1);
assertEqual(prod(double(Ls2(11) > Ls2([2 3 4 5  7 8 9 10  12 13 14 15  17 18 19 20]))),1);
assertEqual(prod(double(Ls2(16) > Ls2([2 3 4 5  7 8 9 10  12 13 14 15  17 18 19 20]))),1);
end

function test_corner_detect
A0= [0 0 0 0;
     0 1 1 0;
     0 1 1 0;
     0 0 0 0]; 
C0=[2 2; 2 3; 3 3; 3 2];
A1= [0 0 0 0 0;
     0 1 1 1 0;
     0 1 0 1 0;
     0 1 1 1 0;
     0 0 0 0 0];
C1=[2 2; 2 3; 2 4; 3 4; 4 4; 4 3; 4 2; 3 2];
A2= [0 0 0 0 0 0 0 0;
     0 1 1 1 1 1 1 0;
     0 1 0 0 0 0 1 0;
     0 1 0 0 0 0 1 0;
     0 1 0 0 0 0 1 0;
     0 1 0 0 0 0 1 0;
     0 1 1 1 1 1 1 0;
     0 0 0 0 0 0 0 0]; 
C2=[2 2; 2 3; 2 4; 2 5; 2 6; 2 7; 3 7; 4 7; 5 7; 6 7; 7 7; 7 6; 7 5; 7 4; 7 3; 7 2; 6 2; 5 2; 4 2; 3 2];
window_size=3; % [x o x] —¼‘¤1ŒÂ‚ðŒ©‚é
assertEqual(Img.corner_detect(C0,window_size),[2 2; 2 3; 3 3; 3 2]);
assertEqual(Img.corner_detect(C1,window_size),[2 2; 2 4; 4 4; 4 2]);
assertEqual(Img.corner_detect(C2,window_size),[2 2; 2 7; 7 7; 7 2]);
assertEqual(Img.corner_detect(C2,window_size,3),[2 2; 2 7; 7 7]);
assertEqual(Img.corner_detect(C2,window_size,2),[2 2; 2 7]);
assertEqual(Img.corner_detect(C2,window_size,1),[2 2]);
end

function test_cell_forall
assertTrue(Util.cell_forall({true true true},@(x) x));
assertFalse(Util.cell_forall({true false true},@(x) x));
end

function test_cell_exists
assertTrue(Util.cell_exists({false false true false},@(x) x));
assertFalse(Util.cell_exists({false false false},@(x) x));
end

function test_bounding_box_of_polygon
A1= [0 0 0 0 0;
     0 1 1 1 0;
     0 1 0 1 0;
     0 1 1 1 0;
     0 0 0 0 0];
C1=[2 2; 2 3; 2 4; 3 4; 4 4; 4 3; 4 2; 3 2];
A=Img.bounding_box_of_polygon(C1);
assertTrue(A(1)==2 && A(2)==2 && A(3)==4 && A(4)==4);
end

function test_bounding_box_contains
A0= [0 0 0 0;
     0 1 1 0;
     0 1 1 0;
     0 0 0 0]; 
C0=[2 2; 2 3; 3 3; 3 2];
A1= [0 0 0 0 0;
     0 1 1 1 0;
     0 1 0 1 0;
     0 1 1 1 0;
     0 0 0 0 0];
C1=[2 2; 2 3; 2 4; 3 4; 4 4; 4 3; 4 2; 3 2];
A2= [0 0 0 0 0 0 0 0;
     0 1 1 1 1 1 1 0;
     0 1 0 0 0 0 1 0;
     0 1 0 0 0 0 1 0;
     0 1 0 0 0 0 1 0;
     0 1 0 0 0 0 1 0;
     0 1 1 1 1 1 1 0;
     0 0 0 0 0 0 0 0]; 
C2=[2 2; 2 3; 2 4; 2 5; 2 6; 2 7; 3 7; 4 7; 5 7; 6 7; 7 7; 7 6; 7 5; 7 4; 7 3; 7 2; 6 2; 5 2; 4 2; 3 2];
assertTrue(Img.bounding_box_contains(C1,C0));
assertTrue(Img.bounding_box_contains(C2,C1));
assertTrue(Img.bounding_box_contains(C2,C0));
assertFalse(Img.bounding_box_contains(C1,C2));
assertFalse(Img.bounding_box_contains(C0,C1));
Cs={C0 C1 C2};
assertTrue(Util.cell_exists(Cs, @(y) Img.bounding_box_contains(y,C0)));
assertTrue(Util.cell_exists(Cs, @(y) Img.bounding_box_contains(y,C1)));
assertFalse(Util.cell_exists({C0 C1}, @(y) Img.bounding_box_contains(y,C2)));
% assertEqual(Util.filter_cell(Cs,@(x) Util.cell_exists(Cs, @(y) Img.bounding_box_contains(y,x))),{C1,C2});
end

function test_decode_template1
T8=[0 0 1 0 0 0 0 0 0 0 0 0 1 1 0 0];
T8err=[1 0 1 0 0 0 0 0 0 0 0 0 1 1 0 0];
assertEqual(Marker.decode_template1(T8),8);
assertEqual(Marker.decode_template1(T8err),-1);

end


function test_decode_template1_with_rotation
M=Marker.generate_markers1;
for I=1:9
    T=M(:,:,I);
    T=T(:)';
    assertEqual(Marker.decode_template1_with_rotation(T),I);
end
T8=[0 0 1 0 0 0 0 0 0 0 0 0 1 1 0 0];
T8err=[1 0 1 0 0 0 0 0 0 0 0 0 1 1 0 0];
T8rot270=[1 0 0 0 1 0 0 0 0 0 0 1 0 0 0 0];
assertEqual(Marker.decode_template1_with_rotation(T8),8);
[id,rot]=Marker.decode_template1_with_rotation(T8rot270);
assertEqual(id,8);
assertEqual(rot,3);
% err correction ‚È‚µ‚Ì‚Æ‚«
% assertEqual(Marker.decode_template1_with_rotation(T8err),-1);
% Te1=[0 0 0 0 1 0 1 0 0 0 0 0 0 0 0 0];
% Te2=[0 0 0 0 0 0 0 0 0 0 0 1 1 0 0 0];
% Te3=[0 0 1 0 0 0 0 1 0 0 0 0 0 0 0 0];
% assertEqual(Marker.decode_template1_with_rotation(Te1),-1);
% assertEqual(Marker.decode_template1_with_rotation(Te2),-1);
% assertEqual(Marker.decode_template1_with_rotation(Te3),-1);
end

function test_bounds_contain
assertEqual(Img.bounds_contain([1 1],[0 0; 0.5 0.5; 1.0 1.0]'),false);
assertEqual(Img.bounds_contain([2 2],[1 1; 0.5 0.5; 2 2]'),true);
assertEqual(Img.bounds_contain([1 1],[0 0; 0.5 0.5; 1.0 1.1]'),false);
assertEqual(Img.bounds_contain([1 1],[-0.1 0; 0.5 0.5; 1.0 1.0]'),false);
assertEqual(Img.bounds_contain([1 1],[0 0; 0.5 0.5; 1.1 1.0]'),false);
assertEqual(Img.bounds_contain([1 1],[0 -0.1; 0.5 0.5; 1.0 1.0]'),false);
assertEqual(Img.bounds_contain([480 640],[0 624]'),false);
end

function test_circle_shift
A=reshape(1:16,4,4);
assertEqual(circshift(A,[0 0]),Util.circle_shift(A,[0 0]));
assertEqual(circshift(A,[1 0]),Util.circle_shift(A,[1 0]));
assertEqual(circshift(A,[2 0]),Util.circle_shift(A,[2 0]));
assertEqual(circshift(A,[0 1]),Util.circle_shift(A,[0 1]));
assertEqual(circshift(A,[0 2]),Util.circle_shift(A,[0 2]));
assertEqual(circshift(A,[-1 0]),Util.circle_shift(A,[-1 0]));
assertEqual(circshift(A,[-2 0]),Util.circle_shift(A,[-2 0]));
assertEqual(circshift(A,[1 1]),Util.circle_shift(A,[1 1]));
assertEqual(circshift(A,[1 2]),Util.circle_shift(A,[1 2]));
assertEqual(circshift(A,[4 2]),Util.circle_shift(A,[4 2]));
assertEqual(circshift(A,[8 2]),Util.circle_shift(A,[8 2]));
end



