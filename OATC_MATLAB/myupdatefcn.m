function txt = myupdatefcn(~,event_obj)
% Customizes text of data tips

pos = get(event_obj,'Position');
txt = {['t (us): ',num2str(pos(1))],['Signal (V): ',num2str(pos(2))]};
