function [Time,MeanVoltage,StartVoltage,PeakVoltage,SettleVoltage] = MeanVoltage(inputArg1,inputArg2,inputArg3,inputArg4)
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here
Time1=inputArg1(:,1);
voltageraw1=inputArg1(:,2);
MeanVoltage1=medfilt1(voltageraw1);

Time2=inputArg2(:,1);
voltageraw2=inputArg2(:,2);
MeanVoltage2=medfilt1(voltageraw2);

Time3=inputArg3(:,1);
voltageraw3=inputArg3(:,2);
MeanVoltage3=medfilt1(voltageraw3);

Time=(Time1+Time2+Time3)/3;
MeanVoltage=(MeanVoltage1+MeanVoltage2+MeanVoltage3)/3;

voltageAvg=0;

    for x=1:10
        voltageAvg=MeanVoltage(x,1)+voltageAvg;
    end
    
StartVoltage=voltageAvg/10;
PeakVoltage=max(MeanVoltage);
SettleVoltage=0.683;

SettleMax=0.683+(0.683-StartVoltage)*0.05;
SettleMin=0.683-(0.683-StartVoltage)*0.05;

plot(Time, MeanVoltage,LineWidth=1)
yline(0.683)
yline(SettleMax,'--')
yline(SettleMin,'--')
legend('Data','Ref','Error band','Location','southeast');
grid on
title(inputArg4)
xlabel("Tid [s]")
ylabel("Spænding [V]")

DeltaStep=SettleVoltage-StartVoltage
DeltaOvershoot=PeakVoltage-SettleVoltage

Forhold=100*DeltaOvershoot/DeltaStep
end
