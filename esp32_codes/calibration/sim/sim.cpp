#include "Arduino.h"
unsigned long g_ms=0,g_us=0; int pinState[64]; void(*isrs[64])(); int duty[8];
#include "../../dual_motor_controller_FINAL.txt"

// ---- plant, built from measured calibration tables (truth = tables scaled by `gain`) ----
struct Plant {
  const double*sp; const double*pw; int n; double gain;
  int nativeDir;           // native-decode count direction when LPWM drives (A:-1, B:+1)
  int c1,c2;               // pins
  double v=0, theta=0;     // deg/s, deg (clockwise positive == LPWM drive)
  long cnt=0; int seq=0; double cpr;
  double plugViolMax=0;
  double ssSpeed(double p){ // |pwm| -> steady speed
    if(p<=pw[0]) return 0; // below kinetic floor
    for(int i=1;i<n;i++) if(p<=pw[i]){double f=(p-pw[i-1])/(pw[i]-pw[i-1]);return gain*(sp[i-1]+f*(sp[i]-sp[i-1]));}
    return gain*sp[n-1];
  }
  void step(double dt,int rp,int lp){
    double d=lp-rp; // + = clockwise
    double tgt; double tau=0.02;
    if(rp==0&&lp==0){tgt=0;tau=0.025;}
    else {
      double a=fabs(d);
      double stat=40.0;
      bool moving=fabs(v)>3;
      if(!moving && a<stat) tgt=0;
      else tgt=(d>=0?1:-1)*ssSpeed(a);
      if(a<35) tgt=0;
      if(tgt*v<0 && fabs(v)>30) plugViolMax=std::max(plugViolMax,fabs(v)); // opposed drive while moving fast
    }
    v+=(tgt-v)*(1-exp(-dt/tau)); theta+=v*dt;
    long want=(long)floor(theta*cpr/360.0*nativeDir);
    int stepdir=want>cnt?1:-1;
    while(cnt!=want){ // native +1 : 00->10->11->01
      static const int fwd[4]={0,2,3,1};
      seq=(seq+stepdir+4)%4; cnt+=stepdir;
      int s=fwd[seq]; pinState[c1]=s>>1; pinState[c2]=s&1; isrs[c1]();
    }
  }
};
// tables from calibration (speed deg/s vs pwm) — reuse firmware tables as the "true" curve
Plant PA,PB; std::string keep;
void tick(int n=1){ for(int i=0;i<n;i++){
  g_ms++; g_us+=1000;
  PA.step(0.001, duty[CH_A_RPWM], duty[CH_A_LPWM]); // physical pins: A RPWM=ch0, LPWM=ch1
  PB.step(0.001, duty[CH_B_RPWM], duty[CH_B_LPWM]);
  if(keep.size()&&g_ms%100==0) Serial.feed(keep.c_str());
  loop(); } }
void cmd(const char*c){Serial.feed(c); if(c[0]=='V') keep=c; else if(c[0]=='S') keep="";}
void flush(bool show){ if(show) fputs(Serial.out.c_str(),stdout); Serial.out.clear(); }
void run(int ms,const char*label,int logEveryMs=0){
  for(int i=0;i<ms;i++){tick(); if(logEveryMs&&i%logEveryMs==0) printf("  t=%5d  A: ramp=%6.1f meas=%6.1f true=%6.1f  pwmR/L=%3d/%3d | B: ramp=%6.1f meas=%6.1f true=%6.1f pwmR/L=%3d/%3d\n",i,motA.rampedVelDegS,motA.measuredVelDegS,PA.v,duty[0],duty[1],motB.rampedVelDegS,motB.measuredVelDegS,PB.v,duty[2],duty[3]);} }
int main(int argc,char**argv){
  PA={FF_SPEED_A,FF_PWM_A,FF_N_A,1.0,-1,ENC_A_C1,ENC_A_C2}; PA.cpr=5070;
  PB={FF_SPEED_B,FF_PWM_B,FF_N_B,1.0,+1,ENC_B_C1,ENC_B_C2}; PB.cpr=5070;
  double gA=argc>1?atof(argv[1]):1.0, gB=argc>2?atof(argv[2]):1.0; PA.gain=gA; PB.gain=gB;
  setup(); tick(300); flush(false);
  const char*mode=argc>3?argv[3]:"all";
  auto T=[&](const char*n){printf("\n=== %s ===\n",n);};
  if(!strcmp(mode,"step")||!strcmp(mode,"all")){
    T("V1 30 / V2 30 from rest (polarity + step)"); cmd("VV 30"); run(1500,"",150); flush(true);
    printf("settled: A meas=%.1f true=%.1f  B meas=%.1f true=%.1f  fault A/B=%d/%d\n",motA.measuredVelDegS,PA.v,motB.measuredVelDegS,PB.v,motA.fault,motB.fault);
    T("step 30->200 deg/s"); cmd("VV 200"); run(1500,"",150); flush(true);
    T("reverse 200 -> -200"); PA.plugViolMax=PB.plugViolMax=0; cmd("VV -200"); run(2000,"",100); flush(true);
    printf("plugging (opposed PWM while |v|>30) max |v|: A=%.1f B=%.1f  fault A/B=%d/%d\n",PA.plugViolMax,PB.plugViolMax,motA.fault,motB.fault);
    T("stop S from -200"); cmd("S"); run(800,"",50); flush(true);
    T("accel timing 0 -> 180"); cmd("VV 180"); 
    int t90=-1; for(int i=0;i<2000;i++){tick(); if(t90<0&&PA.v>=162)t90=i;} printf("A reaches 90%% of 180 at %d ms\n",t90);
    cmd("S"); run(1000,""); flush(false);
  }
  if(!strcmp(mode,"low")||!strcmp(mode,"all")){
    T("low speeds from rest"); for(double v:{10.0,15.0,20.0,30.0,50.0}){ char b[40]; snprintf(b,40,"VV %g",v); cmd(b); run(2500,""); double a=0,bb=0; int n=0; for(int i=0;i<1000;i++){tick(); a+=PA.v; bb+=PB.v; n++; }
      printf("cmd %5.1f  -> true A %.1f B %.1f  fault %d/%d\n",v,a/n,bb/n,motA.fault,motB.fault); cmd("S"); run(1200,""); } flush(false);
  }
  if(!strcmp(mode,"rev")){
    T("true reversal +150 -> -150"); cmd("VV 150"); run(1200,""); PA.plugViolMax=PB.plugViolMax=0; cmd("VV -150"); run(1200,"",60); flush(false);
    printf("opposed-PWM-while-|v|>30 max|v|: A=%.1f B=%.1f fault=%d/%d\n",PA.plugViolMax,PB.plugViolMax,motA.fault,motB.fault);
    cmd("S"); run(800,""); flush(false);
  }
  if(!strcmp(mode,"fault")){
    T("wrong encoder polarity on A (flip nativeDir)"); PA.nativeDir=+1; cmd("V1 60"); run(1500,"",100); flush(true);
    printf("fault A=%d B=%d  pwm A=%d/%d\n",motA.fault,motB.fault,duty[0],duty[1]);
  }
  if(!strcmp(mode,"oldgains")){
    T("OLD gains Kp2.5 Ki8 on same plant"); cmd("K1 2.5 8 0.02"); cmd("K2 2.5 8 0.02"); cmd("VV 100"); run(1500,"",100); flush(false);
  }
  return 0;
}
