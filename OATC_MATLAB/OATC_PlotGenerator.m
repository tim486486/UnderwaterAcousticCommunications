clear all;

%globals
pre_crop_time = 0;
crop_time = 0;

normalize = 0;
drift_correction = 1.000000;
propagation_delay = 1.1; %adjustment for the propagation delay between nodes 1 & 2 (us)

temp_fixed = 10;        %deg C
distance_fixed = 120;   %m
v_sound_fixed = 1450;   %m/s

latitude = 47; %degrees North
saltwater = 1;
if saltwater == 1
    salinity = 0.035; %salinity (%)
else
    salinity = 0; %salinity (%)
end

%Set up the plot window
fig = figure;
fig.NumberTitle = 'off';
fig.Color = [0 0.1 0.2];
fig.InvertHardcopy = 'off';
dcm_obj1 = datacursormode(fig);
set(dcm_obj1,'DisplayStyle','datatip','SnapToDataVertex','off','Enable','on','UpdateFcn',@myupdatefcn);
plots = gobjects(1,4);
titles = gobjects(1,4);
t_delay = zeros(1,2);
y_limit = 1 + 127*(-normalize + 1);

for i=1:2

    % Read the file
    [file,path] = uigetfile;
    fileID = fopen(fullfile(path,file));
    if (i == 1) 
        file1 = file;
    end
    if (i == 2) 
        file2 = file;
    end

    % Data Collection
    header = cell2mat(textscan(fileID,'%d',6,'Delimiter',','));
    num_samples = double(header(1));
    frequency = double(header(2));
    distance = max(double(header(3)),0.01);
    if distance > 100
        distance = 120;
    end
    depth = double(header(4));
    num_synchs = double(header(5));
    sequence = double(header(6));
    
    header = cell2mat(textscan(fileID,'%d',4*(num_synchs+1),'Delimiter',','));
    for j = 1:num_synchs
        dt1(j) = double(header(2 + 4*(j-1)));
        dt2(j) = double(header(4 + 4*(j-1)));
    end

    transmitter = header(4*num_synchs + 1);
    t_end_tr = double(header(4*num_synchs + 2));
    receiver = header(4*num_synchs + 3);
    t_end_re = double(header(4*num_synchs + 4));
    
    data_t = [];
    data_r = [];
    while (~feof(fileID))
        buf = cell2mat(textscan(fileID,'%d',11,'Delimiter',','));
        if numel(buf) > 0
            if buf(1) == transmitter
                data_t = vertcat(data_t,double(buf(2:11)));
            end
            if buf(1) == receiver
                data_r = vertcat(data_r,double(buf(2:11)));
            end
        end
    end
    
    %define the sampling parameters:
    T_sample = 26;
    
    %determine drift correction, the node with the lowest id sets the
    %master time
    dt = dt1./dt2;
    dt = dt(dt > 0.9 & dt < 1.1);
    drift = mean(dt)*drift_correction;
    if transmitter < receiver
        t_end_re = (t_end_re + propagation_delay)*drift;
        T_tr = T_sample;
        T_re = 0.5*T_sample*drift;
    else
        t_end_tr = (t_end_tr + propagation_delay)*drift;
        T_tr = T_sample*drift;
        T_re = 0.5*T_sample;
    end
    
    %set up the transmitter time axis, adjusted for drift:
    t_start_tr = t_end_tr - T_tr*(num_samples - 1);
    t_tr = t_start_tr : T_tr : t_end_tr;
    
    %set up the receiver time axis:
    t_start_re = t_end_re - T_re*(num_samples - 1);
    t_re = t_start_re : T_re : t_end_re;
    
    %set up the master time axis:
    up_sample = 2*T_sample;
    T_sample = T_sample / up_sample;
    t_start = t_start_tr;
    t_end = round(t_end_re);
    t = t_start : T_sample : t_end;
    
    %process transmitter data:
    data_t = myNormalize(data_t,normalize);
    mean_t = mean(data_t);
    data_t = interp1(t_tr,data_t,t)';
    data_t(isnan(data_t)) = mean_t; 
    tr_l_shift = 50*up_sample;
    data_t_shift = [data_t(tr_l_shift+1:numel(data_t)) ; mean_t*ones(tr_l_shift,1)];  
    
    %process receiver data:
    data_r = 255 - data_r;
    data_r = myNormalize(data_r,normalize);
    mean_r = mean(data_r);
    data_r = interp1(t_re,data_r,t)';    
    data_r(isnan(data_r)) = mean_r;
    if (crop_time ~= 0)
        if (mod(i,2) ~= 0) 
            crop_point = round((crop_time - t_start) / T_sample);
        end
        if crop_point > 0
            data_r(crop_point:end) = 0;
        end
    end
    if (pre_crop_time ~= 0)
        if (mod(i,2) ~= 0) 
            pre_crop_point = round((pre_crop_time - t_start) / T_sample);
        end
        if pre_crop_point > 0
            data_r(1:pre_crop_point) = 0;
        end
    end
    
    %apply matched filtering
    [acor,lag] = xcorr(data_r,data_t_shift);
    [~,I] = max(acor);
    sample_delay = lag(I);
    t_delay(i) = T_sample*(sample_delay - tr_l_shift);
    acor = acor(tr_l_shift+1:end);
    lag = lag(1:numel(acor));
    
    %shift the transmitted data, for visualization of the cross correlation
    data_t_corr = [mean(data_t_shift)*ones(abs(sample_delay),1) ; data_t_shift(1:(numel(t) - abs(sample_delay)))];
    
    %plots the raw signals on top:
    subplot(3,2,1 + (i-1));
    plot(t,data_t,'LineWidth',1, 'Color', 'g');
    plots(1 + 3*(i-1)) = gca;
    axis([-inf,inf,-y_limit,y_limit]);
    xlabel('System Time (\mus)');
    ylabel('Data');
    titles(1 + 3*(i-1)) = title(strcat('Node', {' '}, num2str(transmitter), ' Transmitted'), 'Color', 'w');
    grid on;
    hold on;
    plot(t,data_r,'LineWidth',1,'Color', 'r');
    
    %plot the matched filter output:
    subplot(3,2,3 + (i-1));
    plot(lag,acor/max(acor),'LineWidth',1, 'Color', 'g');
    plots(3 + 3*(i-1)) = gca;
    axis([0,numel(data_r),-inf,inf]);
    xlabel('Samples Shifted');
    ylabel('Dot Product');
    titles(3 + 3*(i-1)) = title('Matched Filter Output', 'Color', 'w');
    grid on;

    %plot the correlated signals next:
    subplot(3,2,5 + (i-1));
    plot(t(end - 0.5*up_sample*num_samples:end),data_t_corr(end - 0.5*up_sample*num_samples:end),'LineWidth',1, 'Color', 'g');
    plots(2 + 3*(i-1)) = gca;
    axis([-inf,inf,-y_limit,y_limit]);
    xlabel('System Time (\mus)');
    ylabel('Data');
    titles(2 + 3*(i-1)) = title(strcat('Strongest Arrival, Travel Time =', {' '}, num2str(t_delay(i)), ' \mus'), 'Color', 'w');
    grid on;
    hold on;
    plot(t(end - 0.5*up_sample*num_samples:end),data_r(end - 0.5*up_sample*num_samples:end),'LineWidth',1,'Color', 'r');
       
    for j = 1 + 3*(i-1):3 + 3*(i-1)
        plots(j).FontSize = 12;
        plots(j).FontName = 'Century Gothic';
        plots(j).FontWeight = 'bold';
        plots(j).Color = [0 0 0];
        plots(j).GridColor = 'w';
        plots(j).GridAlpha = 0.8;
        plots(j).XColor = 'w';
        plots(j).YColor = 'w';
        titles(j).FontSize = 16;
    end

    fclose(fileID);   
end

t1 = t_delay(1);
t2 = t_delay(2);
dt = t2 - t1;
st = t2 + t1;

distance_measured = v_sound_fixed*st*1E-6/2;
v_sound_measured_1 = (distance_fixed/2)*st/(t1*t2*1E-6);
v_sound_measured_2 = getSpeedLeroy(salinity,depth,latitude,temp_fixed);
v_water_measured_1 = 100*dt*(distance_fixed/2)/(t1*t2*1E-6);
v_water_measured_2 = 1E-4*dt*(v_sound_fixed^2)/(2*distance_fixed);
temp_measured = getTempLeroy(salinity,depth,latitude,v_sound_fixed);

X = [distance_measured 
    v_sound_measured_1 
    v_sound_measured_2 
    temp_measured];

X_ = [distance_fixed 
    v_sound_fixed 
    v_sound_fixed 
    temp_fixed];

header = sgtitle(['Sequence: ', num2str(dec2bin(sequence)), ', '...
    '\Sigmat: ', num2str(st/2,'%3.2f'), '\mus, '...
    'Distance: ', num2str(distance_measured,'%3.2f'), ' m, ',...    
    'c_1: ', num2str(v_sound_measured_1,'%3.0f'), ' m/s, ',...
    'c_2: ', num2str(v_sound_measured_2,'%3.0f'), ' m/s, ',...
    'temp: ', num2str(temp_measured,'%3.2f'), ' \circC, ',...
    '\Deltat: ', num2str(dt,'%3.2f'), '\mus, '...
    'v_1: ', num2str(v_water_measured_1,'%3.1f'), ' cm/s, ',...
    'v_2: ', num2str(v_water_measured_2,'%3.1f'), ' cm/s, ']);

header.Color = 'w';
header.FontSize = 12;
header.FontName = 'Century Gothic';
header.FontWeight = 'bold';
window_title = strcat('Left Graphs: ', {' '}, file1, ', Right Graphs: ', {' '}, file2);
fig.Name = string(window_title);
fig.WindowState = 'maximized';
figure(fig);

function txt = myupdatefcn(~,event_obj)
    % Customizes text of data tips
    pos = get(event_obj,'Position');
    txt = {['x: ',num2str(pos(1))],['y: ',num2str(pos(2))]};
end

function A_ = myNormalize(A,normalize)
    if normalize == 1
        A = (A - min(A)) / (max(A) - min(A)); 
    end
    A = A - mean(A);
    A_ = A;    
end

function temp = getTempLeroy(sa,de,lat,c)
    %Equation from Reference 24
    a = [1402.5        5             -5.44e-2      2.1e-4...
        1.33*sa       -1.23e-2*sa     8.7e-5*sa          ...
        1.56e-2*de     2.55e-7*de^2  -7.3e-12*de^3       ...
        1.2e-6*de*(lat-45)           -9.5e-13*de^3       ...
        3e-7*de        1.43e-5*sa*de];
    
    p = [a(4)...
    a(3) + a(7) + a(13)...
    a(2) + a(6) + a(12)...
    a(1) + a(5) + a(8) + a(9) + a(10) + a(11) + a(14) - c];

    r = roots(p);
    temp = r(imag(r)==0);
end

function c = getSpeedLeroy(sa,de,lat,temp)
    %Equation from Reference 24
    a = [1402.5        5             -5.44e-2      2.1e-4...
        1.33*sa       -1.23e-2*sa     8.7e-5*sa          ...
        1.56e-2*de     2.55e-7*de^2  -7.3e-12*de^3       ...
        1.2e-6*de*(lat-45)           -9.5e-13*de^3       ...
        3e-7*de        1.43e-5*sa*de];
    
    c = a(4)*temp^3 + ...
    (a(3) + a(7) + a(13))*temp^2 + ...
    (a(2) + a(6) + a(12))*temp + ...
    a(1) + a(5) + a(8) + a(9) + a(10) + a(11) + a(14);
end

function temp = getTempMackenzie(sa,de,~,c)
    %Equation from Reference 23
    a = [1448.96       4.591            -5.304e-2...
        2.374e-4       1.34*(sa-35)      1.630e-2*de...
        1.675e-7*de^2 -1.025e-2*(sa-35) -7.139e-13*de^3];
    
    p = [a(4)...
        a(3)...
        a(2)+a(8)+a(9)...
        a(1)+a(5)+a(6)+a(7)-c];

    r = roots(p);
    temp = r(imag(r)==0);
end

function c = getSpeedMackenzie(sa,de,~,temp)
    %Equation from Reference 22
    a = [1448.96       4.591            -5.304e-2...
        2.374e-4       1.34*(sa-35)      1.630e-2*de...
        1.675e-7*de^2 -1.025e-2*(sa-35) -7.139e-13*de^3];
    
    c = a(4)*temp^3 + ...
        a(3)*temp^2 + ...
        (a(2) + a(8) + a(9))*temp + ...
        a(1)+a(5)+a(6)+a(7);
end

