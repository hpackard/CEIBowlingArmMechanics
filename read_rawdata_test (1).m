%% read data section 
setupJsonFileName = 'C:\Users\kwinings\Desktop\SignLanguageData\CalibrationRawData\test1.setup.json';
% rawDataFileName = 'RawData_DBF_1pl';
% radarCubeDataFileName = 'RCData_DBF_1pl';
% rawDataFileName = 'RawData_DBF_1pr-Amin';
% radarCubeDataFileName = 'RCData_DBF_1pr-Amin';
rawDataFileName = 'RawData_DBF_2plr';
radarCubeDataFileName = 'RCData_DBF_2plr';
%read data command
NewFileName = 'C:\Users\kwinings\Desktop\SignLanguageData\CalibrationRawData\test1_Raw_0.bin';
rawDataReader(setupJsonFileName, rawDataFileName, radarCubeDataFileName, 0, NewFileName)
load(rawDataFileName)

%% show the range map of the collected data

load(radarCubeDataFileName)
NChip = radarCube.dim.numChirps;
NFram = radarCube.dim.numFrames;
NRan = radarCube.dim.numRangeBins;
ChData = zeros(NRan,NChip*NFram);
c = 3e8;
fc = radarCube.rfParams.startFreq+radarCube.rfParams.bandwidth/2;
PRT = radarCube.rfParams.framePeriodicity/1000;
PRF = 1./PRT;
lambda = c/fc;
d = lambda/2;
% the beforming angle, positive means point left 
%cta = -(linspace(-10,10,numel(radarCube.data))*0-25)/180*pi;
% cta = -(linspace(-10,10,numel(radarCube.data))+30)/180*pi;
 cta = (linspace(-10,10,numel(radarCube.data)))*0;
% aa = [1 exp(1i*2*pi/lambda*d*cos(theta)) exp(1i*2*pi/lambda*d*2*cos(theta)) exp(1i*2*pi/lambda*d*3*cos(theta))].';
% aa = [1 exp(1i*pi*sin(theta)) exp(1i*2*pi*sin(theta)) exp(1i*3*pi*sin(theta))].';
FRawDate = 1;
for i =1:numel(radarCube.data)
    theta = cta(i);
    aa = [1 exp(1i*pi*sin(theta)) exp(1i*2*pi*sin(theta)) exp(1i*3*pi*sin(theta))].';
    if FRawDate
        tmp = cell2mat(adcRawData.data(i));
        tmp = single(squeeze(permute(tmp,[3,1,2])));
        tmp = tmp - ( tmp >=2.^15).* 2.^16;
        rawData8 = reshape(tmp, [8, length(tmp)/8]);
        rawDataI = rawData8(1:4,:).';
        rawDataQ = rawData8(5:8,:).';
        rawData= rawDataI+ 1i*rawDataQ;
    else
        tmp = cell2mat(radarCube.data(i));
        tmp = squeeze(permute(tmp,[3,1,2]));
        rawData = tmp;
    end
       %ChData(:,(1:NChip)+(i-1)*NChip) = rawData*aa; % Do beamforming
        ChData(:,(1:NChip)+(i-1)*NChip) = rawData(:,3); % Dosen't do
%        beamforming

    
end
ChData0 = ChData - mean(ChData,2);
if FRawDate
    ChData0 = fft(ChData0,[],1);
end
Tcor = (0:size(ChData0,2))*PRT;
Rcor = (0:127)*radarCube.rfParams.rangeResolutionsInMeters;
figure;imagesc(Tcor,Rcor,(abs(ChData0(1:128,:))))
%figure;imagesc(db(abs(ChData0(1:128,:))),[50 90])
xlabel('Time(s)')
ylabel('Range(m)')
xlim([0 5])


%set(gca, 'YDir','normal')
     
%set(gca,'xtick',[],'ytick',[])
%frame = frame2im(getframe(gca));
%imwrite(frame,['C:\Users\kwinings\Desktop\bowlingtest\bowlingRange5', '.png']);
% ChData0 = ChData0(:,4001:12000);
% ChData0 = ChData0(:,1:25000);
% ChData0 = ChData0(:,10001:25000);





%% Doppler spectrum 
WinL = 256; % window size 
Step = 31; % window moving step size
[Nr,Na] = size(ChData0);
Naa = fix((Na-WinL)/Step)+1;
DopSpec = zeros(WinL,Naa);
Rind = (1:2); % the range where you are, to exclude the noise.
for i=1:Naa
    % DopSpec(:,i) = fftshift(fft(sum(ChData0(54:66,(1:WinL)+(i-1)*OL),1).*hamming(WinL)')); 
     DopSpec(:,i) = fftshift(fft(sum(ChData0(Rind,(1:WinL)+(i-1)*Step),1).*hamming(WinL)'));
    
end

DopSpec = DopSpec - DopSpec(:,1); %Remove DC Noise without MTI Filter
Tcor = (0:size(DopSpec,2))*PRT*Step;
DopCor = (-WinL/2:WinL/2-1)/WinL*PRF;
DopSpec = DopSpec./max(abs(DopSpec(:)));
figure;imagesc(Tcor,DopCor,db(abs(flipud(DopSpec))));
colormap(jet(256));
caxis([-40 0]);

xlabel('Time(s)')
ylabel('Doppler(Hz)')

set(gca, 'YDir','normal')
     
set(gca,'xtick',[],'ytick',[])
frame = frame2im(getframe(gca));
imwrite(frame,['C:\Users\kwinings\Desktop\SignLanguageData\CalibrationPlots\test1', '.png']);
%xlim([0 15])


