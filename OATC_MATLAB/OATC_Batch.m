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

selpath = uigetdir(path);

testfiledir = selpath;
matfiles = dir(fullfile(testfiledir, '*.m'));
nfiles = length(matfiles);

all_data_points = zeros(1,nfiles);
all_frequency = zeros(1,nfiles);
all_distance = zeros(1,nfiles);
all_depth = zeros(1,nfiles);
all_sequence = zeros(1,nfiles);
all_transmitter = zeros(1,nfiles);
all_end_time_t = zeros(1,nfiles);
all_receiver = zeros(1,nfiles);
all_end_time_r = zeros(1,nfiles);
all_data_t = [];
all_data_r = [];
travel_times = zeros(3,nfiles);
drift = zeros(1,nfiles);
dt = zeros(1,ceil(nfiles/2));
v_1 = zeros(1,ceil(nfiles/2));

% Data Collection
for i = 1 : nfiles   
    
    data_file = fopen(fullfile(testfiledir, matfiles(i).name));
    header = cell2mat(textscan(data_file,'%d',6,'Delimiter',','));
    all_data_points(i) = double(header(1));
    all_frequency(i) = double(header(2));
    all_distance(i) = max(double(header(3)),0.01);
    all_depth(i) = double(header(4));
    if i == 1
        num_synchs = double(header(5));
        all_dt1 = zeros(num_synchs,nfiles);
        all_dt2 = zeros(num_synchs,nfiles);
    end
    
    all_sequence(i) = double(header(6));
    
    header = cell2mat(textscan(data_file,'%d',4*(num_synchs+1),'Delimiter',','));
    for j = 1:num_synchs
        all_dt1(j,i) = double(header(2 + 4*(j-1)));
        all_dt2(j,i) = double(header(4 + 4*(j-1)));
    end

    all_transmitter(i) = header(4*num_synchs + 1);
    all_end_time_t(i) = double(header(4*num_synchs + 2));
    all_receiver(i) = header(4*num_synchs + 3);
    all_end_time_r(i) = double(header(4*num_synchs + 4));
    
    data_t_buf = [];
    data_r_buf = [];
    while (~feof(data_file))
        buf = cell2mat(textscan(data_file,'%d',11,'Delimiter',','));
        if numel(buf) > 0
            if buf(1) == all_transmitter(i)
                data_t_buf = vertcat(data_t_buf,double(buf(2:11)));
            end
            if buf(1) == all_receiver(i)
                data_r_buf = vertcat(data_r_buf,double(buf(2:11)));
            end
        end
    end
    
    all_data_t(:,i) = data_t_buf;
    all_data_r(:,i) = data_r_buf;    
    fclose(data_file);
end

for i = 1:nfiles
    matfiles(i).name
    num_samples = all_data_points(i);
    frequency = all_frequency(i);
    distance = all_distance(i);
    if distance > 100
        distance = 120;
    end
    depth = all_depth(i);
    dt1 = all_dt1(:,i);
    dt2 = all_dt2(:,i);
    transmitter = all_transmitter(i);
    t_end_tr = all_end_time_t(i);
    receiver = all_receiver(i);
    t_end_re = all_end_time_r(i);
    data_t = all_data_t(:,i);
    data_r = all_data_r(:,i);

    %define the sampling parameters:
    T_sample = 26;
    
    %determine drift correction, the node with the lowest id sets the
    %master time
    dt_ = dt1./dt2;
    dt_ = dt_(dt_ > 0.9 & dt_ < 1.1);
    drift(i) = mean(dt_)*drift_correction;
    if transmitter < receiver
        t_end_re = (t_end_re + propagation_delay)*drift(i);
        T_tr = T_sample;
        T_re = 0.5*T_sample*drift(i);
    else
        t_end_tr = (t_end_tr + propagation_delay)*drift(i);
        T_tr = T_sample*drift(i);
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
        if (pre_crop_point > 0)
            data_r(1:pre_crop_point) = 0;
        end
    end
    
    %perform cross correlation
    [acor,lag] = xcorr(data_r,data_t_shift);
    [~,I] = max(acor);
    sample_delay = lag(I);
    t_delay = T_sample*(sample_delay - tr_l_shift);    

    travel_times(1,i) = transmitter;
    travel_times(2,i) = receiver;
    travel_times(3,i) = t_delay; 
end

%Open a file to store calculated cal data
name = strcat('\',erase(matfiles(1).name,'.m'),'_to_',erase(matfiles(nfiles).name,'.m'),'.txt');
fileName = strcat(pwd,name);
batch_file = fopen(fileName,'a');
fprintf(batch_file, 'distance(manual): %3.2d m\n', distance_fixed);
fprintf(batch_file, 'temp(manual): %3.2d deg C\n', temp_fixed);
fprintf(batch_file, 'v sound(manual): %3.0f m/s\n', v_sound_fixed);
fprintf(batch_file, 'salinity: %3.3f %%\n', salinity);
fprintf(batch_file, 'latitude: %3.0f degrees North\n', latitude);
fprintf(batch_file, 'crop time: %3.0f us\n', crop_time);
fprintf(batch_file, 'pre crop time: %3.0f us\n', pre_crop_time);
fprintf(batch_file, 'drift correction adjustment: %3.8f\n', drift_correction);
fprintf(batch_file, 'propagation delay: %3.3f us\n', propagation_delay);

j = 1;
for i = 1:2:nfiles-1    
    t1 = travel_times(3,i);
    t2 = travel_times(3,i+1);
    depth = all_depth(i);
    sequence = all_sequence(i);
    
    st(j) = t2+t1;    
    d(j) = v_sound_fixed*st(j)*1E-6/2;
    c_1(j) = (distance_fixed/2)*st(j)/(t1*t2*1E-6);
    c_2(j) = getSpeedLeroy(salinity,depth,latitude,temp_fixed);
    temp(j) = getTempLeroy(salinity,depth,latitude,v_sound_fixed); 
    
    dt(j) = t2-t1;
    v_1(j) = 100*dt(j)*(distance_fixed/2)/(t1*t2*1E-6);    
    v_2(j) = 1E-4*dt(j)*(v_sound_fixed^2)/(2*distance_fixed);
    
    %Transmission Information
    fprintf(batch_file, '%s, %s, ', matfiles(i).name, matfiles(i+1).name);
    fprintf(batch_file, 'depth: %1.0f cm, ', depth);
    fprintf(batch_file, 'sequence: %s, ', dec2bin(sequence));  
    fprintf(batch_file, '%1.0g->%1.0g: %3.1f us, ', travel_times(1,i), travel_times(2,i), travel_times(3,i)); 
    fprintf(batch_file, '%1.0g->%1.0g: %3.1f us, ', travel_times(1,i+1), travel_times(2,i+1), travel_times(3,i+1));   
    fprintf(batch_file, 'drift correction: %3.4f%%, ', 100*(drift(i) - 1));
    %Travel Time Sum Calculations
    fprintf(batch_file, 'st: %3.2f us, ', st(j)/2);
    fprintf(batch_file, 'd: %3.2f m, ', d(j));
    fprintf(batch_file, 'c1: %3.0f m/s, ', c_1(j));
    fprintf(batch_file, 'c2: %3.0f m/s, ', c_2(j));
    fprintf(batch_file, 'temp: %3.2f deg C, ', temp(j));
    %Travel Time Difference Calculations
    fprintf(batch_file, 'dt: %3.2f us, ', dt(j));
    fprintf(batch_file, 'v1: %3.1f cm/s, ', v_1(j));
    fprintf(batch_file, 'v2: %3.1f cm/s\n', v_2(j));    
    j = j + 1;
end
fprintf(batch_file, 'average st: %3.2f us\n', mean(rmoutliers(st)));
fprintf(batch_file, 'average d: %3.2f m\n', mean(rmoutliers(d)));
fprintf(batch_file, 'average c1: %3.0f m/s\n', mean(rmoutliers(c_1)));
fprintf(batch_file, 'average c2: %3.0f m/s\n', mean(rmoutliers(c_2)));
fprintf(batch_file, 'average temp: %3.2f deg C\n', mean(rmoutliers(temp)));
fprintf(batch_file, 'average dt: %3.2f us\n', mean(rmoutliers(dt)));
fprintf(batch_file, 'average v1: %3.1f cm/s\n', mean(rmoutliers(v_1)));
fprintf(batch_file, 'average v2: %3.1f cm/s\n\n', mean(rmoutliers(v_2)));
fclose(batch_file);

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
