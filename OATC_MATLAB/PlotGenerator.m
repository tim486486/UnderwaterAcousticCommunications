close all
clear all

[file,path] = uigetfile;
file = fullfile(path,file)
fileID = fopen(file);
formatSpec = '%d';
D = fscanf(fileID, formatSpec);

samples = D(1);         %number of samples
f_s = D(2)/13;          %ADC sampling frequency (Hz)
distance = D(3);

samplePeriod = 1E6/f_s;

x_begin(1) = 5;
y_begin(1) = 5 + samples + 1;
x_begin(2) = 5 + 2*samples + 2;
y_begin(2) = 5 + 3*samples + 3;

xTime(1) = D(x_begin(1) - 1);
yTime(1) = D(y_begin(1) - 1);
xTime(2) = D(x_begin(2) - 1);
yTime(2) = D(y_begin(2) - 1);

x1 = D(x_begin(1) : x_begin(1) + samples - 1)';
y1 = D(y_begin(1) : y_begin(1) + samples - 1)';
x2 = D(x_begin(2) : x_begin(2) + samples - 1)';
y2 = D(y_begin(2) : y_begin(2) + samples - 1)';

t_start(1) = xTime(1) - (samples*samplePeriod);
t_end(1) = yTime(1);
t_start(2) = xTime(2) - (samples*samplePeriod);
t_end(2) = yTime(2);

t1 = (t_start(1) : samplePeriod : t_end(1) - samplePeriod);
t2 = (t_start(2) : samplePeriod : t_end(2) - samplePeriod);

x1 = x1/max(x1);                                    % normalize
x1 = x1 + (0.5 - x1(1));                            % shift
x1 = [x1 zeros(1,numel(t1) - numel(x1))];           % pad

y1 = y1/max(y1);                                    % normalize
y1 = 512 - y1;                                      % invert
y1 = y1 + (0.5 - y1(1));                            % shift
y1 = [zeros(1,numel(t1) - numel(y1)), y1];          % pad

x2 = x2/max(x2);                                    % normalize
x2 = x2 + (0.5 - x2(1));                            % shift
x2 = [x2 zeros(1,numel(t2) - numel(x2))];           % pad

y2 = y2/max(y2);                                    % normalize
y2 = 512 - y2;                                      % invert
y2 = y2 + (0.5 - y2(1));                            % shift
y2 = [zeros(1,numel(t2) - numel(y2)), y2];          % pad

%Set up the plot window
fig = figure;
fig.NumberTitle = 'off';
fig.Name = 'OATC Data';
fig.Color = [0 0.1 0.2];
fig.InvertHardcopy = 'off';
dcm_obj1 = datacursormode(fig);
set(dcm_obj1,'DisplayStyle','datatip','SnapToDataVertex','off','Enable','on','UpdateFcn',@myupdatefcn);

plots = gobjects(1,2);
titles = gobjects(1,2);
legends = gobjects(1,2);

%Node 1 to Node 2
subplot(211);
plot(t1,x1,'LineWidth',1, 'Color', 'g');
plots(1) = gca;
axis([-inf,inf,0,1]);
xlabel('System Time (\mus)');
ylabel('Data (Normalized and Shifted)');
titles(1) = title('Node 1 to Node 2', 'Color', 'w');
hold on
plot(t1,y1,'LineWidth',1,'Color', 'r');
grid on
legends(1) = legend('Node 1 (T)', 'Node 2 (R)');

%Node 1 to Node 2
subplot(212);
plot(t2,x2,'LineWidth',1, 'Color', 'g');
plots(2) = gca;
axis([-inf,inf,0,1]);
xlabel('System Time (\mus)');
ylabel('Data (Normalized and Shifted)');
titles(2) = title('Node 2 to Node 1', 'Color', 'w');
hold on
plot(t2,y2,'LineWidth',1,'Color', 'r');
grid on
legends(2) = legend('Node 2 (T)', 'Node 1 (R)');

for i=1:2
    plots(i).FontSize = 12;
    plots(i).FontName = 'Century Gothic';
    plots(i).FontWeight = 'bold';
    plots(i).Color = [0 0 0];
    plots(i).GridColor = 'w';
    plots(i).GridAlpha = 0.8;
    plots(i).XColor = 'w';
    plots(i).YColor = 'w';
    titles(i).FontSize = 20;
    legends(i).Color = [0.1 0.1 0.1];
    legends(i).TextColor = 'w';
    legends(i).EdgeColor = 'w';
end

fclose(fileID);