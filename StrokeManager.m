classdef StrokeManager < handle
    
properties
hMuran;
hStrokeStateRemote;
end
    
methods

function obj=StrokeManager(hMuran)
obj.hMuran=hMuran;
obj.hStrokeStateRemote=utotch.muran.StrokeStateRemote;
end

function b=isConnected(obj)
b=obj.hStrokeStateRemote.isConnected;    
end

function connect(obj)
obj.hStrokeStateRemote.connect;
end

function TX=extractTarget(obj)
N=obj.hMuran.numRobots;    
SX=obj.hStrokeStateRemote.pollNormal;
SN=size(SX,2);
if SN < N 
    TX=[];
    return;
end
TX=zeros(2,N);
for I=1:(N-1)
TX(:,I)=SX(:,floor(SN/(N-1))*(I-1)+1);
end
TX(:,end)=SX(:,end);    
end

function updateTarget(obj)
fprintf(2,'[StrokeUpdate]isConnected=%d, isChanged=%d\n',obj.isConnected,obj.hStrokeStateRemote.isChanged);
if obj.isConnected && obj.hStrokeStateRemote.isChanged
    TX=obj.extractTarget;
    assert(length(TX)==obj.hMuran.numRobots || isempty(TX));
    if length(TX)==obj.hMuran.numRobots
        obj.hMuran.hFormationManager.doFormationAll(TX);
    end
    obj.hStrokeStateRemote.clearPoints;
end

end

end

end

