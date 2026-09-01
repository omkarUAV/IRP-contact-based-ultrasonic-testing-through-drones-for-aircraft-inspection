%% complete_fixed_vs_scheduled_pid_validation.m
% Complete paired validation: Fixed PID vs Gain-Scheduled PID
% Every hold-out geometry and disturbance is included. No favourable cases
% are selected. The same reference, initial condition, plant, actuator
% limits, disturbance and sensor-noise sequence are used for each pair.
%
% NOTE: the plant is assumed for simulation. Gains are preselected; the
% common score J is an evaluation/design score, not numerical optimisation.

clear; clc; close all;

%% 1) Common conditions
Ts=0.02; Tend=10; t=0:Ts:Tend; N=numel(t);              % 50 Hz
Fref=5.5; F0=5.0; tol=0.5; Flow=5.0; Fhigh=6.0;
uMin=0; uMax=0.012; vMax=0.020;                        % actuator limits
sigmaF=0.015; fc=8; alpha=exp(-2*pi*fc*Ts);             % sensor model
nRep=30; baseSeed=310826;

%% 2) Controllers
fixed=[1.00e-3 3.00e-3 5.00e-5];                       % [Kp Ki Kd]
rhoVertex=[0 0.5 1];
KpV=[0.85e-3 1.00e-3 1.18e-3];
KiV=[2.80e-3 3.10e-3 3.50e-3];
KdV=[4.00e-5 5.00e-5 6.00e-5];

%% 3) Unseen hold-out geometries
rhoH=[0.20 0.35 0.65 0.80];                            % not tuning vertices
nS=numel(rhoH);

%% 4) Disturbances
dName=["Nominal","+1 N pulse","-1 N pulse","0.75 N 2 Hz sinusoid"];
dEnd=[NaN 5.5 5.5 7.0];
nD=numel(dName);
recoveryDwell=0.50;

%% 5) Common evaluation score (same for both controllers)
% Lower is better. This is not an optimiser.
w=[0.35 0.25 0.15 0.20 0.05];  % RMSE, Peak, Recovery, Tolerance, Saturation
recoveryNorm=2.0; failPenalty=1.5;

%% 6) Storage: dimensions = surface x disturbance x repetition x controller
% controller 1 = Fixed, controller 2 = Scheduled
RMSE=nan(nS,nD,nRep,2);
Peak=nan(nS,nD,nRep,2);
Recovery=nan(nS,nD,nRep,2);
RecFail=false(nS,nD,nRep,2);
TolPct=nan(nS,nD,nRep,2);
SatPct=nan(nS,nD,nRep,2);
J=nan(nS,nD,nRep,2);

repF=cell(nS,nD); repS=cell(nS,nD);                     % repetition 1 only

%% 7) Complete paired test matrix
for s=1:nS
    rho=rhoH(s);
    [Kplant,tau]=plantModel(rho);

    sched=[interp1(rhoVertex,KpV,rho,'linear'), ...
           interp1(rhoVertex,KiV,rho,'linear'), ...
           interp1(rhoVertex,KdV,rho,'linear')];

    for d=1:nD
        disturbance=makeDisturbance(t,d);

        for r=1:nRep
            % EXACTLY the same measurement-noise vector for the controller pair
            rng(baseSeed+10000*s+100*d+r,'twister');
            noise=sigmaF*randn(1,N);

            out1=simPID(t,Ts,Fref,F0,Kplant,tau,fixed,disturbance,...
                        noise,alpha,uMin,uMax,vMax);
            out2=simPID(t,Ts,Fref,F0,Kplant,tau,sched,disturbance,...
                        noise,alpha,uMin,uMax,vMax);

            M1=metrics(t,out1,Fref,Flow,Fhigh,uMin,uMax,dEnd(d),recoveryDwell);
            M2=metrics(t,out2,Fref,Flow,Fhigh,uMin,uMax,dEnd(d),recoveryDwell);

            [RMSE(s,d,r,1),Peak(s,d,r,1),Recovery(s,d,r,1),...
             RecFail(s,d,r,1),TolPct(s,d,r,1),SatPct(s,d,r,1)] = unpack(M1);
            [RMSE(s,d,r,2),Peak(s,d,r,2),Recovery(s,d,r,2),...
             RecFail(s,d,r,2),TolPct(s,d,r,2),SatPct(s,d,r,2)] = unpack(M2);

            J(s,d,r,1)=score(M1,tol,recoveryNorm,failPenalty,w);
            J(s,d,r,2)=score(M2,tol,recoveryNorm,failPenalty,w);

            if r==1
                repF{s,d}=out1; repS{s,d}=out2;          % predetermined
            end
        end
    end
end

%% 8) Raw-results table: no cases removed
ctrlName=["Fixed PID","Scheduled PID"];
Controller=strings(2*nS*nD*nRep,1); Rho=zeros(size(Controller));
Disturbance=strings(size(Controller)); Repeat=zeros(size(Controller));
RMSE_F=zeros(size(Controller)); PeakError=zeros(size(Controller));
RecoveryTime=nan(size(Controller)); RecoveryFailed=false(size(Controller));
TimeInTolerance=zeros(size(Controller)); Saturation=zeros(size(Controller));
ObjectiveJ=zeros(size(Controller));

q=0;
for s=1:nS
 for d=1:nD
  for r=1:nRep
   for c=1:2
    q=q+1;
    Controller(q)=ctrlName(c);
    Rho(q)=rhoH(s); Disturbance(q)=dName(d); Repeat(q)=r;
    RMSE_F(q)=RMSE(s,d,r,c); PeakError(q)=Peak(s,d,r,c);
    RecoveryTime(q)=Recovery(s,d,r,c); RecoveryFailed(q)=RecFail(s,d,r,c);
    TimeInTolerance(q)=TolPct(s,d,r,c); Saturation(q)=SatPct(s,d,r,c);
    ObjectiveJ(q)=J(s,d,r,c);
   end
  end
 end
end
RawResults=table(Controller,Rho,Disturbance,Repeat,RMSE_F,PeakError,...
    RecoveryTime,RecoveryFailed,TimeInTolerance,Saturation,ObjectiveJ);
writetable(RawResults,'fixed_vs_scheduled_raw_results.csv');

%% 9) Median/IQR summary across 30 repetitions
nCase=nS*nD;
RhoS=zeros(nCase,1); DistS=strings(nCase,1);
FixRM=zeros(nCase,1); SchRM=zeros(nCase,1); FixRMIQR=zeros(nCase,1); SchRMIQR=zeros(nCase,1);
FixPk=zeros(nCase,1); SchPk=zeros(nCase,1);
FixRec=nan(nCase,1); SchRec=nan(nCase,1); FixFail=zeros(nCase,1); SchFail=zeros(nCase,1);
FixTol=zeros(nCase,1); SchTol=zeros(nCase,1);
FixSat=zeros(nCase,1); SchSat=zeros(nCase,1); FixJ=zeros(nCase,1); SchJ=zeros(nCase,1);

q=0;
for s=1:nS
 for d=1:nD
  q=q+1; RhoS(q)=rhoH(s); DistS(q)=dName(d);
  a=squeeze(RMSE(s,d,:,1)); b=squeeze(RMSE(s,d,:,2));
  FixRM(q)=median(a); SchRM(q)=median(b); FixRMIQR(q)=iqrLocal(a); SchRMIQR(q)=iqrLocal(b);
  FixPk(q)=median(squeeze(Peak(s,d,:,1))); SchPk(q)=median(squeeze(Peak(s,d,:,2)));
  FixTol(q)=median(squeeze(TolPct(s,d,:,1))); SchTol(q)=median(squeeze(TolPct(s,d,:,2)));
  FixSat(q)=median(squeeze(SatPct(s,d,:,1))); SchSat(q)=median(squeeze(SatPct(s,d,:,2)));
  FixJ(q)=median(squeeze(J(s,d,:,1))); SchJ(q)=median(squeeze(J(s,d,:,2)));
  FixFail(q)=100*mean(squeeze(RecFail(s,d,:,1)));
  SchFail(q)=100*mean(squeeze(RecFail(s,d,:,2)));
  if d>1
      ar=squeeze(Recovery(s,d,:,1)); br=squeeze(Recovery(s,d,:,2));
      ar=ar(isfinite(ar)); br=br(isfinite(br));
      if ~isempty(ar), FixRec(q)=median(ar); end
      if ~isempty(br), SchRec(q)=median(br); end
  end
 end
end

RMSE_Improvement_pct=100*(FixRM-SchRM)./FixRM;
Peak_Improvement_pct=100*(FixPk-SchPk)./FixPk;
Tolerance_Change_pp=SchTol-FixTol;
Saturation_Change_pp=SchSat-FixSat;
J_Improvement_pct=100*(FixJ-SchJ)./FixJ;

Summary=table(RhoS,DistS,FixRM,SchRM,FixRMIQR,SchRMIQR,...
    RMSE_Improvement_pct,FixPk,SchPk,Peak_Improvement_pct,...
    FixRec,SchRec,FixFail,SchFail,FixTol,SchTol,Tolerance_Change_pp,...
    FixSat,SchSat,Saturation_Change_pp,FixJ,SchJ,J_Improvement_pct);

disp(Summary);
writetable(Summary,'fixed_vs_scheduled_summary.csv');

%% 10) Force-response plots: predetermined repetition 1 for ALL 16 cases
for s=1:nS
    figure('Color','w','Name',sprintf('Hold-out rho %.2f',rhoH(s)));
    tiledlayout(2,2,'TileSpacing','compact','Padding','compact');
    for d=1:nD
        nexttile;
        plot(t,repF{s,d}.F,'LineWidth',1.25); hold on;
        plot(t,repS{s,d}.F,'--','LineWidth',1.25);
        yline(Fref,'k:'); yline(Flow,'k--'); yline(Fhigh,'k--');
        grid on; xlabel('Time (s)'); ylabel('Force (N)'); title(dName(d));
        if d==1, legend('Fixed PID','Scheduled PID','Reference','Location','best'); end
    end
    sgtitle(sprintf('Predetermined repetition 1, unseen hold-out \\rho_g=%.2f',rhoH(s)));
end

%% 11) Complete metric plots
labels=compose('\\rho %.2f-D%d',RhoS,repmat((1:nD)',nS,1));
x=1:nCase;

plotBars(x,[FixRM SchRM],labels,'Median force RMSE (N)',...
    'RMSE: all hold-out conditions');
plotBars(x,[FixPk SchPk],labels,'Median peak error (N)',...
    'Peak force error: all hold-out conditions');
plotBars(x,[FixTol SchTol],labels,'Time within 5.0-6.0 N (%)',...
    'Time within force tolerance: all hold-out conditions');

figure('Color','w'); recMask=isfinite(FixRec)|isfinite(SchRec);
bar(find(recMask),[FixRec(recMask) SchRec(recMask)]); grid on;
ylabel('Median recovery time (s)'); title('Recovery after disturbance');
legend('Fixed PID','Scheduled PID','Location','best');
xticks(find(recMask)); xticklabels(labels(recMask)); xtickangle(45);

plotBars(x,[FixSat SchSat],labels,'Actuator saturation time (%)',...
    'Actuator saturation: all hold-out conditions');
plotBars(x,[FixJ SchJ],labels,'Common normalised score J',...
    'Same evaluation objective applied to both controllers');

%% 12) Critical audit: explicitly expose unfavourable cases
fprintf('\n================ CRITICAL PERFORMANCE AUDIT ================\n');
fprintf('Scheduled lower RMSE:       %d/%d cases\n',sum(SchRM<FixRM),nCase);
fprintf('Scheduled lower peak error: %d/%d cases\n',sum(SchPk<FixPk),nCase);
fprintf('Scheduled higher tolerance: %d/%d cases\n',sum(SchTol>FixTol),nCase);
fprintf('Scheduled lower saturation: %d/%d cases\n',sum(SchSat<FixSat),nCase);
fprintf('Scheduled lower score J:    %d/%d cases\n',sum(SchJ<FixJ),nCase);

cmpRec=isfinite(FixRec)&isfinite(SchRec);
fprintf('Scheduled faster recovery:  %d/%d comparable disturbed cases\n',...
    sum(SchRec(cmpRec)<FixRec(cmpRec)),sum(cmpRec));

bad=find(SchRM>=FixRM);
if isempty(bad)
    fprintf('No hold-out case produced higher median RMSE for Scheduled PID.\n');
else
    fprintf('\nCases where Scheduled PID does NOT reduce median RMSE:\n');
    for k=bad'
        fprintf('rho=%.2f | %s | Fixed=%.4f N | Scheduled=%.4f N\n',...
            RhoS(k),DistS(k),FixRM(k),SchRM(k));
    end
end

[best,bestI]=max(RMSE_Improvement_pct);
[worst,worstI]=min(RMSE_Improvement_pct);
fprintf('\nLargest RMSE improvement: %.2f%% | rho=%.2f | %s\n',...
    best,RhoS(bestI),DistS(bestI));
fprintf('Smallest/worst RMSE change: %.2f%% | rho=%.2f | %s\n',...
    worst,RhoS(worstI),DistS(worstI));

fprintf(['\nInterpretation: superiority should be claimed only if the scheduled ' ...
    'controller improves the full test matrix consistently without trading ' ...
    'lower RMSE for unacceptable peak error, recovery failure or saturation.\n']);
fprintf('Recovery failures remain in the raw table and are penalised in J.\n');

%% =============================== FUNCTIONS ==============================
function [K,tau]=plantModel(rho)
% Assumed, not experimentally identified.
K=900-120*rho; tau=0.20+0.04*rho;
end

function d=makeDisturbance(t,c)
d=zeros(size(t));
if c==2, d((t>=5)&(t<5.5))=1.0;
elseif c==3, d((t>=5)&(t<5.5))=-1.0;
elseif c==4
    m=(t>=5)&(t<7); d(m)=0.75*sin(2*pi*2*(t(m)-5));
end
end

function o=simPID(t,Ts,Fref,F0,K,tau,g,d,noise,a,uMin,uMax,vMax)
N=numel(t); F=zeros(1,N); Fm=zeros(1,N); u=zeros(1,N); x=zeros(1,N);
F(1)=F0; uEq=Fref/K; x(1)=min(max(uEq,uMin),uMax);
Fm(1)=F(1)+noise(1); I=0; prev=Fm(1);
for k=1:N-1
    if k>1, raw=F(k)+noise(k); Fm(k)=a*Fm(k-1)+(1-a)*raw; end
    e=Fref-Fm(k); D=-(Fm(k)-prev)/Ts;
    us=uEq+g(1)*e+g(2)*I+g(3)*D;
    u(k)=min(max(us,uMin),uMax);
    if (us>=uMin&&us<=uMax)||(us>uMax&&e<0)||(us<uMin&&e>0), I=I+e*Ts; end
    dx=min(max(u(k)-x(k),-vMax*Ts),vMax*Ts); x(k+1)=x(k)+dx;
    F(k+1)=F(k)+Ts*((-F(k)+K*x(k)+d(k))/tau);
    prev=Fm(k);
end
Fm(end)=a*Fm(end-1)+(1-a)*(F(end)+noise(end)); u(end)=u(end-1);
o.F=F; o.Fm=Fm; o.u=u; o.x=x;
end

function M=metrics(t,o,Fref,lo,hi,uMin,uMax,dEnd,dwell)
e=o.F-Fref; M.rmse=sqrt(mean(e.^2)); M.peak=max(abs(e));
inside=(o.F>=lo)&(o.F<=hi); M.tol=100*mean(inside);
M.sat=100*mean((abs(o.u-uMin)<1e-10)|(abs(o.u-uMax)<1e-10));
M.rec=NaN; M.fail=false;
if isfinite(dEnd)
    M.rec=recovery(t,o.F,lo,hi,dEnd,dwell);
    M.fail=~isfinite(M.rec);
end
end

function tr=recovery(t,F,lo,hi,t0,dwell)
tr=NaN; i0=find(t>=t0,1); n=max(1,ceil(dwell/(t(2)-t(1))));
inside=(F>=lo)&(F<=hi);
for k=i0:numel(t)-n+1
    if all(inside(k:k+n-1)), tr=t(k)-t0; return; end
end
end

function [a,b,c,d,e,f]=unpack(M)
a=M.rmse; b=M.peak; c=M.rec; d=M.fail; e=M.tol; f=M.sat;
end

function J=score(M,tol,rNorm,failPenalty,w)
r=M.rmse/tol; p=M.peak/tol; ti=1-M.tol/100; s=M.sat/100;
if M.fail, rec=failPenalty; elseif isfinite(M.rec), rec=M.rec/rNorm; else, rec=0; end
J=w(1)*r+w(2)*p+w(3)*rec+w(4)*ti+w(5)*s;
end

function q=pctl(x,p)
x=sort(x(:)); n=numel(x); pos=1+(n-1)*p/100; lo=floor(pos); hi=ceil(pos);
if lo==hi, q=x(lo); else, q=x(lo)+(pos-lo)*(x(hi)-x(lo)); end
end

function r=iqrLocal(x)
r=pctl(x,75)-pctl(x,25);
end

function plotBars(x,Y,labels,ytext,ttl)
figure('Color','w'); bar(x,Y); grid on; ylabel(ytext); title(ttl);
legend('Fixed PID','Scheduled PID','Location','best');
xticks(x); xticklabels(labels); xtickangle(45);
end
