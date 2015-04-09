function loop_sample()
userData.continue=true;
hFig=figure(1);
set(hFig,'UserData',userData);
set(hFig,'KeyPressFcn',{@onKeyPressed});
while(true);
    disp('loop...');
    userData=get(hFig,'UserData');
    if (~userData.continue)
        break;
    end
    pause(1);
end
end


function onKeyPressed(src,event)
disp('onKeyPressed');
userData=get(src,'UserData');
userData.continue=false;
set(src,'UserData',userData);
end

