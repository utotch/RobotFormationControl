classdef FormationManager < handle
properties(Constant)
% Default9=[-0.5  0 0.5 -0.5 0 0.5 -0.5 0 0.5;
%           0.5 0.5 0.5  0  0  0   -0.5 -0.5 -0.5];
Default9=[-1  0 1 -1 0 1 -1 0 1;
         1 1 1  0  0  0   -1 -1 -1];
Side45=[-1 -1 -1 -1  1 1 1 1 1;
         0.75 0.25 -0.25 -0.75  0.95 0.5 0 -0.5 -0.95];
Side54=[-1 -1 -1 -1 -1  1 1 1 1;
         0.95 0.5 0 -0.5 -0.95  0.75 0.25 -0.25 -0.75];
Dance=[-1 -1 -1 -1 1 1 1 1 0;
       0.75 0.25 -0.25 -0.75 0.75 0.25 -0.25 -0.75 0]
CharS=[0.5 0 -0.5 -0.25 0 0.25 0.5 0 -0.5;
       0.5 0.75 0.5 0.25 0 -0.25 -0.5 -0.75 -0.5];
Tetris=[-0.9 0.6 0.9 -0.9 -0.6 0.9 -0.9 0.6 0.9;
        0.5 0.5 0.5 0 0 0 -0.5 -0.5 -0.5];
Cross=[-1 1 -0.5 0.5 0 -0.5 0.5 -1 1;
       1 1 0.5 0.5 0 -0.5 -0.5 -1 -1];
Triangle=[0 -0.25 0.25 -0.75 0.75 -1 -0.4 0.4 1;
          1 0.4 0.4 -0.4 -0.4 -1 -1 -1 -1];
    
end
    
properties
hMuran;
end

methods 
    
function obj=FormationManager(hMuran)
obj.hMuran=hMuran;    
end
    
function doFormationAll(obj,targetX)
    T=obj.planTarget(obj.hMuran.robotStateNormal([1 2],:),targetX);
    for J=1:9
        id=T(J);
        obj.hMuran.setTargetPointNormal(id,targetX(:,J));
    end
end

function T=planPresetTarget(obj,X,formation)
if strcmpi(formation,'default9')
    targetX=FormationManager.Default9;
elseif strcmpi(formation,'side45')
    targetX=FormationManager.Side45;
elseif strcmpi(formation,'side54')
    targetX=FormationManager.Side54;
elseif strcmpi(formation,'s')
    targetX=FormationManager.CharS;
end
T=obj.planTarget(X,targetX);
end

function T=planTarget(obj,X,targetX)
assert(size(X,1)==2 && size(X,2)==9);
assert(size(targetX,1)==2 && size(targetX,2)==9);
table=zeros(2,9,9);
for id=1:9
    table(:,:,id)=repmat(targetX(:,id),1,9);
end
T=zeros(1,9);
D=zeros(9,9);
for id=1:9
    Y=table(:,:,id)-X;
    [~,D(id,:)]=sort(Y(1,:).^2+Y(2,:).^2);
end

T(1)=D(1,1);
for id=2:9
    for J=1:9
        tid=D(id,J);
        if ~ismember(tid,T(1:(id-1)))
            T(id)=tid;
            break;
        end
    end
end

end



end
    
end

