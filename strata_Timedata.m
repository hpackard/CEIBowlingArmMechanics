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
mmicConfig = strata.config.timedata();
sequence = strata.sequence.timedata();
stages = strata.stages.timedata();

%% configure all settings
boardRadar.setConfiguration(mmicConfig);
boardRadar.setSequence(sequence);
boardRadar.setProcessingStages(stages);
boardRadar.setup();

%% Extra configuration
radarRxs = boardRadar.getIRadarRxs(0);
registers = radarRxs.getIRegisters();
radarRxs.enableDividerOutput(true); % enable divider output for test
registers.setRegisterBits('0x042C', '0x0001'); % DMUX1 as output
registers.writeRegister('0x0434', '0x0020'); % DMUX1 map DMUX_A

%% Example configuration to use DMUX pin as Ramp Scenario trigger source
%trigger_source_dmux = 2; % DMUX2
%registers.clearRegisterBits('0x042C', '0x0002'); % DMUX2 as input
%radarRxs.setTriggerSource(trigger_source_dmux);
%radarPins = radarRxs.getIRadarPins();
%radarPins.configureDmuxPin(trigger_source_dmux, 6); % GPIO_MODE_OUTPUT_PUSH_PULL
%radarPins.setDmuxPin(trigger_source_dmux, 0);


%% Get values for visualization
radarInputInfo = boardRadar.getInputInfo();
samples = radarInputInfo.samples;
ramps = radarInputInfo.ramps;
bitWidth = radarInputInfo.bitWidth;
NoRX = radarInputInfo.rxChannels;
NoTX = radarInputInfo.txChannels;

% Enabled RIF channels
if (~isnumeric(sequence.rxMask))
    rxMask = strata.utils.convertToNumber(sequence.rxMask);
else
    rxMask = sequence.rxMask;
end
% (if rif1_ch2 is enabled rif1_ch1 has to be enabled aswell even if corresponding RX1 chain in RXS is disabled)
EnabledRX = bitget(rxMask,1:32);

if radarInputInfo.modulation == 0
    ramps = ramps / double(NoTX);
end
%% start measurements and do one
boardRadar.startMeasurements();
boardRadar.doMeasurement();

%% Example how to use DMUX pin as Ramp Scenario trigger source
%radarPins.setDmuxPin(trigger_source_dmux, 1); % trigger Ramp Scenario execution
%radarPins.setDmuxPin(trigger_source_dmux, 0);


%% show timedata
[measurement, timeStamp] = board.getFrameBuffer();

%Converting measurement into int16 array
measurement = typecast(measurement(1:end), 'int16');
measurement = measurement(1:2:end); %take the real part
measurement = reshape(measurement, samples, NoRX, NoTX, ramps);
sif = permute(measurement, [2 1 3 4]);

cnt = 1;
for idx_rx=1:1:NoRX
    
    if EnabledRX(idx_rx)
        %****************************************************
        %    sif(TX-idx,RX-idx,NumberOfSamples,NumberOfRamps)
        %    scaled to fullscale +-1
        %****************************************************
        for idx_tx=1:1:NoTX
            figure(cnt);
            clf;
            hold on;
            plot(mean(squeeze(sif(idx_rx, :,idx_tx,:)),2),'r');
            plot(squeeze(sif(idx_rx,:,idx_tx,1)),'b');
            plot(squeeze(sif(idx_rx,:,idx_tx,2:1:end)),'k');
            plot(mean(squeeze(sif(idx_rx, :,idx_tx,:)),2),'r');%plotting again; for visibility in figure
            hold off;
            grid on;
            box on;
            legend('Mean value','First ramp','Other ramps');
            title(['RX' num2str(idx_rx) ' / TX' num2str(idx_tx)]);
            xlabel('Samples (1)');
            ylabel('Amplitude (FS)');
            %ylim([-1 1]);
            cnt=cnt+1;
            drawnow;
        end
    end
end
boardRadar.stopMeasurements();


%%%%%%%%% Destroy the board object %%%%%%%%%
board.delete();

%for debugging, unload the wrapper dll to be able to update it
clear wrapper_matlab
