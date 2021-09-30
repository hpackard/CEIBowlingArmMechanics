%--------------------------------------------------------------------------
% AoA parameters
%--------------------------------------------------------------------------
% Configure
NFFT                =  512;
NFFTAnt             =  256;
NrMIMO              =  8;       % Number of MIMO channels 
min_range           =   1;  % minimum calculated range in meters
max_range           =   10;  % maximum calculated range in meters
%%idk if the stuff above this comment is what i can change to change the range...i think it is!!!!

% do not change
c0                  =   3 * 10^8;   % speed of light
full_scale          =   2^15-1;     % full scale Aurix data 
win2D               =   repvec(hanning(cfgRadarData.samples),NrMIMO);
scale_win           =   sum(win2D(:,1));
k_ramp              =   cfg.rmpCfg.f_delta / cfg.rmpCfg.t_ramp;
range               =   (0:NFFT-1)'./NFFT.*(50*10^6 / getDecimation(cfg.rxCfg.RX_settings)).*c0/(2.*k_ramp);    % fADC should be variable
[~, min_range_idx]  =   min(abs(range - min_range));
[~, max_range_idx]  =   min(abs(range - max_range));
range_view            =   range(min_range_idx : max_range_idx);

% Window function for  NrMIMO virtual receive channels
winAnt              =   hanning(NrMIMO);
scale_winAnt        =   sum(winAnt);
winAnt2D            =   repmat(winAnt.',numel(range_view),1);
angle_deg           =   (-NFFTAnt/2 : NFFTAnt/2-1)'./NFFTAnt.*180;

% Positions for polar plot of cost function
vU                  =   linspace(-1, 1, NFFTAnt);
[mRange , mU]       =   ndgrid(range_view, vU);
mX                  =   mRange.*mU;
mY                  =   mRange.*cos(asin(mU));

% Calibration data
mCalib            =   ones(cfgRadarData.samples, NrMIMO);%repmat(CalData(AntIdx).',N-11,1);


