PH = PH_withShadowing;
rangeRes = 150;
prf = 12e3;
numPulses = 64;

% This helper function forms and plots an RDM from a phase history matrix

c = physconst('lightspeed');

% Form DC-centered RDM and convert to dBW
RDM = fftshift(fft(PH,[],2),2);
RDM = 20*log10(abs(RDM));

% Range and Doppler bins
rngBins = 0:rangeRes:c/(2*prf);
dopBins = -prf/2:prf/numPulses:prf/2-prf/numPulses;

% Plot
imagesc(dopBins,rngBins/1e3,RDM);
set(gca,'ydir','normal')
xlabel('Doppler (Hz)')
ylabel('Range (km)')
mx = max(RDM(:));
if ~isinf(mx)
    clim([mx-60 mx]);
end
colorbar

