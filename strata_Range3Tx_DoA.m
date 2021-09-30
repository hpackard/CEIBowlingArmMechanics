%**************************************************************************
%                        IMPORTANT NOTICE
%**************************************************************************
% THIS SOFTWARE IS PROVIDED "AS IS". NO WARRANTIES, WHETHER EXPRESS,
% IMPLIED OR STATUTORY, INCLUDING, BUT NOT LIMITED TO, IMPLIED WARRANTIES
% OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE APPLY TO THIS
% SOFTWARE. INFINEON SHALL NOT, IN ANY CIRCUMSTANCES, BE LIABLE FOR SPECIAL,
% INCIDENTAL, OR CONSEQUENTIAL DAMAGES, FOR ANY REASON WHATSOEVER.
%**************************************************************************

clc
clearvars
close all

%% connect to board
board = strata.utils.connectToBoard();
boardRadar = board.getIBoardRadar();

%% get config structs
mmicConfig = strata.config.rangeDoppler();
sequence = strata.sequence.range3Tx_DoA();
stages = strata.stages.rangeData();

%% configure all settings
boardRadar.setConfiguration(mmicConfig);
boardRadar.setSequence(sequence);
boardRadar.setProcessingStages(stages);
boardRadar.setup();

%% Extra configuration                           
radarRxs = boardRadar.getIRadarRxs(0);
radarRxs.enableDividerOutput(true); % enable divider output for test

%% Get values for visualiztion
radarInputInfo = boardRadar.getInputInfo();
samples = radarInputInfo.samples;
ramps = double(radarInputInfo.ramps);
NoTX = double(radarInputInfo.txChannels);
NoRX = double(radarInputInfo.rxChannels);
%NoRange = 2^nextpow2(double(samples))/2;
NoRange = 512;

%% scaling for plot

c0=3e8;
%sweep duration
tsw=(sequence.tRamp-sequence.tRampStartDelay);
%effective RF bandwidth
bsw=(sequence.ramps(1).fDelta/sequence.tRamp*(tsw));
%pulse repetition time
PRT=sequence.tRamp+sequence.tJump+sequence.tWait;
if sequence.modulation == 0
    PRT=PRT/NoTX;
    ramps=ramps/NoTX;
end
%maximum velocity
f_sin = sequence.ramps(1).fStart+sequence.ramps(1).fDelta/2;
vmax=c0/2/(f_sin)/PRT/2;
rmax=mmicConfig.sampleRate*tsw/bsw*c0/2/2;

r_vect=linspace(0,0.5-1/NoRange,NoRange)*rmax*2;
v_vect=linspace(-0.5,0.5-1/ramps,ramps)*2*vmax;

FftBinSz = 8; % bytes;
nciBinSz = 4; % bytes;
Fft2Sz = NoRange*ramps*NoRX*NoTX*FftBinSz;
NciSz = NoRange*ramps*nciBinSz;

%% AoA parameters
NFFTAnt             =  256;
NrMIMO              =  8;       % Number of MIMO channels 
min_range           =   0.5;  % minimum calculated range in meters
max_range           =   6;  % maximum calculated range in meters

[~, min_range_idx]  =   min(abs(r_vect - min_range));
[~, max_range_idx]  =   min(abs(r_vect - max_range));
range_view          =   r_vect(min_range_idx : max_range_idx);

% Window function for  NrMIMO virtual receive channels
winAnt              =   hanning(NrMIMO);
scale_winAnt        =   sum(winAnt);
winAnt2D            =   repmat(winAnt.',numel(range_view),1);
winAnt3D            =   permute(repmat(winAnt,1,numel(range_view),ramps),[2 3 1]);
angle_deg           =   (-NFFTAnt/2 : NFFTAnt/2-1)'./NFFTAnt.*180;

% Positions for polar plot of cost function
vU                  =   linspace(-1, 1, NFFTAnt);
[mRange , mU]       =   ndgrid(range_view, vU);
mX                  =   mRange.*mU;
mY                  =   mRange.*cos(asin(mU));

%% RD demo
figure(1);clf;
box on;

%% Setting count variables
x = [];
y = [];
tidx = [];
t = [];
count = 1;


boardRadar.startMeasurements();

% inittime = posixtime(datetime(datestr(now)));
% fprintf('Starting time is %d \n', inittime);

tic;
for idx=1:1:2000000
    idx
    
%     z = posixtime(datetime(datestr(now)));
%     fprintf('%d',z)
    
    [measurement, timeStamp] = board.getFrameBuffer(20000);
    % Data to extract is Tx1 and Tx3 from the linear measurement memory
    % Reshape to get rid of unwanted half of FFT and remap the Rx channels
    RD_data = typecast(measurement, 'int32');
    %RD_data = double(complex(RD_data(1:2:end), RD_data(2:2:end)));
    %plot(abs(RD_data))
    RD_data = double(reshape(complex(RD_data(1:2:end), RD_data(2:2:end)), NoRange*2, []));
    %RD_data = RD_data(1:NoRange,[3 4 1 2 11 12 9 10]);
    RD_data = RD_data(1:NoRange,[1 2 3 4 9 10 11 12]);
    
    % Look at range slices at zero doppler only to compare
    range_profile = RD_data;
    range_profile_FOV = range_profile(min_range_idx : max_range_idx, :);  
    %plot(unwrap(angle(range_profile)));
    spec = fftshift(fft(range_profile_FOV.*winAnt2D,NFFTAnt,2)/scale_winAnt,2);
        
    % normalize cost function
    JdB              =   20.*log10(abs(spec));
    %[JMax, idx_JMax] =   max(JdB(:));
    JMax = 130;
    JNorm            =   JdB - JMax;
    JNorm(JNorm < -20)  =   -20;

    if idx==1
        %polar plot
        figure(1);
        h = surf(mX,mY,JNorm);
        J1 = JNorm;
        shading flat;
        view(0,90);
        xlim([-5 5]);
        ylim([0 20]);
        caxis([-20 -8]);
        axis equal; xlabel('x (m)'); ylabel('y (m)'); colormap('jet');
    
%       fileID = fopen('radartest.txt','w+');
    end
    
    if ~ishandle(h)
        break;
    end
    
    JNew = JNorm - J1 - 20;       %eliminate nonmoving objects
    JNew(JNew < -20)  =   -20;
    set(h,'ZData',JNew);

    drawnow;
  
%     maxJNew = max(JNew(:));      %find maximum intensity of this index
%     [row,col] = find(JNew == maxJNew);   
%     xcoord = mX(row,col);
%     ycoord = mY(row,col);
%     dlmwrite('radartest.txt', JNew,'-append','newline','pc')

    if (max(JNew(:))>-17)
        [i,j] = find(JNew>-17);       
        row = round(mean(i));         
        col = round(mean(j));
        %time(count) = posixtime(datetime(datestr(now)));
        tidx(count) = idx/20;
        t(count) = idx/20 - tidx(1);
        x(count) = mX(row,col);
        y(count) = mY(row,col);
        %if((x(count) > x(count - 1) + 1) || (x(count) < x(count-1) - 1))        
        count = count + 1;
        %end
    end
    
end
timer1 = toc

t1 = t;
x1 = x;
y1 = y;

boardRadar.stopMeasurements();

%%%%%%%%% Destroy the board object %%%%%%%%%
board.delete();

%for debugging, unload the wrapper dll to be able to update it
clear wrapper_matlab

radar_height = 44; %inches
rad_h = radar_height/39.37; %meters

len = length(x);

if(x(1) > -0.6)  %First Point Negative
    x = x(2:len);
    y = y(2:len);
    t = t(2:len);
end

len = length(x);

% if(x(1) > x(2))  %First LEss Than Second
%    x(1:(len-1)) = x(2:len);
%    y(1:(len-1)) = y(2:len);
%    t(1:(len-1)) = t(2:len);   
%    len = len-1;
% end

le = len;

for(k = 1:(le-1))  %Any Point Lower than Ground
    if(x(k) > rad_h)
      x(k:len-1) = x(k+1:len); 
      y(k:len-1) = y(k+1:len); 
      t(k:len-1) = t(k+1:len); 
      len= len-1;    
    end
end

if(x(len) > rad_h)  %Last Point Lower than Ground
   x = x(1:len-1); 
   y = y(1:len-1); 
   t = t(1:len-1); 
   len = len-1;
end

if(x(len) < x(1))  %Last Point Higher than First
   x = x(1:len-1); 
   y = y(1:len-1); 
   t = t(1:len-1); 
   len = len-1;
end

le = len;

for(k = 2:(le-1))   %Any Point Higher than First
   if(x(k) < x(1))
      x(k:len-1) = x(k+1:len); 
      y(k:len-1) = y(k+1:len); 
      t(k:len-1) = t(k+1:len); 
      len= len-1;    
   end
end

le = len;

for(k = 2:(le-1))   %Points Past 1 m Similar to First
   if(x(k) < (x(1)+0.15) && y(k) > (y(1)+1))
      x(k:len-1) = x(k+1:len); 
      y(k:len-1) = y(k+1:len); 
      t(k:len-1) = t(k+1:len); 
      len= len-1;    
   end
end

le = len;

for(k = 2:(le-1))   %Middle Lower than Two Adjacent
   if(x(k) > x(k-1) && x(k) > (x(k+1) + 0.1))
      x(k:len-1) = x(k+1:len); 
      y(k:len-1) = y(k+1:len); 
      t(k:len-1) = t(k+1:len); 
      len= len-1;
   end
end

for(k = 2:(le-1))   %Middle Higher than Two Adjacent
   if(x(k) < x(k-1) && x(k) < (x(k+1) - 0.1))
      x(k:len-1) = x(k+1:len); 
      y(k:len-1) = y(k+1:len); 
      t(k:len-1) = t(k+1:len); 
      len= len-1;
   end
end

if(x(len) < x(len - 1))  %Last Point Higher than Second to Last
   x = x(1:(len-1));
   y = y(1:(len-1));
   t = t(1:(len-1));
   len = len-1;
end

    x = x(1:len);
    y = y(1:len);
    t = t(1:len);
    t = t - t(1);

plot(y,-x,'o');
title('Scatterplot of x and y Positional Values');
xlabel('Horizontal distance from radar');
ylabel('Vertical distance from radar');
 
fileID = fopen('Positional_Values.txt','w');
fprintf(fileID,'%f ',y);
fprintf(fileID,'\n');
fprintf(fileID,'%f ',-x);
fprintf(fileID,'\n');
fprintf(fileID,'%f ',t);
fclose(fileID);

finy = LSFit(y,-x,t);
fprintf('\n Final Height is: %g',finy);


fileID = fopen('zonemat_height.txt','r');
formatSpec = '%f';
A = fscanf(fileID,formatSpec);
fclose(fileID);

height = A;

fileID_2 = fopen('heightmat_off_ground.txt','r');
formatSpec = '%f';
B = fscanf(fileID_2,formatSpec);
fclose(fileID_2);

height_off_ground = B;

p1 = [0 0 0+B];
p2 = [431.8 0 0+B];
p3 = [431.8 431.8 0+B];
p4 = [0 431.8 0+B];

p5 = [0 0 height+B];
p6 = [431.8 0 height+B];
p7 = [431.8 431.8 height+B];
p8 = [0 431.8 height+B];

poly_rectangle_mat(p1, p2, p3, p4)
poly_rectangle_mat(p5, p6, p7, p8)
poly_rectangle_mat(p1, p2, p6, p5)
poly_rectangle_mat(p2, p3, p7, p6)
poly_rectangle_mat(p4, p3, p7, p8)
poly_rectangle_mat(p1, p4, p8, p5)

view(120, 43)
%Get pitch height
vert_pitch_dist_from_radar = 1000*(finy);
height_of_radar = rad_h*1000;
pitch_height_from_ground = height_of_radar+vert_pitch_dist_from_radar;

[x,y,z] = sphere;
surf(x*36.5+215.9,y*36.5,z*36.5+pitch_height_from_ground)
xlabel('length (mm)')
ylabel('width (mm)')
zlabel('height (mm)')

top_of_zone = height_off_ground+height;

if (pitch_height_from_ground >= height_off_ground) && (pitch_height_from_ground <= top_of_zone)
    title('Unique Strike Zone: Pitch = Strike!')
end
if(pitch_height_from_ground < height_off_ground)
       title('Unique Strike Zone: Pitch = Ball Low!')
end
if(pitch_height_from_ground > top_of_zone)
    title('Unique Strike Zone: Pitch = Ball High!')    
end

fprintf('\n Final Height off Ground (mm): %g \n',pitch_height_from_ground);

figure
plot(t1,-x1,'o')
title('Original Y Position v Time')
xlabel('Time (s)')
ylabel('Y (m)') 