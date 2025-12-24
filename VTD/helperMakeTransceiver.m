function rdr = helperMakeTransceiver( bw,fc,rangeRes,prf,useCustomElem )
% This helper function creates a radarTransceiver from some basic system
% parameters.

c = physconst('lightspeed');

rdr = radarTransceiver;

rdr.TransmitAntenna.OperatingFrequency = fc;
rdr.ReceiveAntenna.OperatingFrequency = fc;

rdr.Waveform.PRF = prf;

sampleRate = c/(2*rangeRes);
sampleRate = prf*round(sampleRate/prf); % adjust to match constraint with PRF

rdr.Receiver.SampleRate = sampleRate;
rdr.Waveform.SampleRate = sampleRate;

rdr.Waveform.PulseWidth = 2*rangeRes/c;

if isempty(bw)
    % Use an isotropic element
    rdr.TransmitAntenna.Sensor = phased.IsotropicAntennaElement;
    rdr.ReceiveAntenna.Sensor = phased.IsotropicAntennaElement;
else
    % Get the number of elements required to meet the specified beamwidth
    sinc3db = 0.8859;
    N = round(sinc3db*2./(bw(:).'*pi/180));
    N = flip(N);
    lambda = freq2wavelen(fc,c);
    if numel(N) == 1
        % Use a back-baffled ULA
        array = phased.ULA(N,lambda/2);
        array.Element.BackBaffled = true;
    else
        % Use URA
        array = phased.URA(N,lambda/2);
    end

    if useCustomElem
        % Use a custom element corresponding to the sum beam
        az = -180:.4:180;
        el = -90:.4:90;
        G = pattern(array,fc,az,el,'Type','efield','normalize',false);
        M = 20*log10(abs(G));
        P = angle(G);
        E = phased.CustomAntennaElement('FrequencyVector',[fc-1 fc+1],...
            'AzimuthAngles',az,'ElevationAngles',el,'MagnitudePattern',M,'PhasePattern',P);
        rdr.TransmitAntenna.Sensor = E;
        rdr.ReceiveAntenna.Sensor = E;
    else
        rdr.TransmitAntenna.Sensor = array;
        rdr.ReceiveAntenna.Sensor = array;
    end
end

end