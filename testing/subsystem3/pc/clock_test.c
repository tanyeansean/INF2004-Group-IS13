/* Distance clock check: feed left/right encoder count streams of a weaving car through the
   accumulation used in drv_track.c (OLD = |ds| per tick, NEW = signed sum, emitted per count)
   and compare with the true travel of a sensor ARM mm left of the centre line. */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#define K256 (((85-55)*256)/110)
static unsigned clk_old, clk_new, rem_o, rem_n; static int pend;
static void tick(int dl,int dr){
  int ds = dl*256 + (dl-dr)*K256;
  { int a = ds<0?-ds:ds; rem_o += (unsigned)a*820u; clk_old += rem_o>>8; rem_o &= 0xFF; }
  pend += ds;
  if (pend >= 256 || pend <= -256) { int a = pend<0?-pend:pend; rem_n += (unsigned)a*820u; clk_new += rem_n>>8; rem_n &= 0xFF; pend = 0; }
}
static void run(const char*what,double v,double yaw_dps,double arm){
  double xl=0,xr=0,xs=0; int cl=0,cr=0; double worst_o=0,worst_n=0; unsigned lo=0,ln=0; double ls=0;
  clk_old=clk_new=rem_o=rem_n=0; pend=0;
  for(int t=0;t<6000;t++){ /* 3 s, 0.5 ms ticks */
    double w=yaw_dps*3.14159265/180.0*sin(6.2831853*2.5*t*0.0005);
    xl+=(v-55*w)*0.0005; xr+=(v+55*w)*0.0005; xs+=fabs(v-arm*w)*0.0005;
    int nl=(int)floor(xl*1000/82), nr=(int)floor(xr*1000/82);
    tick(nl-cl,nr-cr); cl=nl; cr=nr;
    if(t%60==59){ /* every 30 ms window (about one narrow bar): rate error */
      double ds_true=(xs-ls)*10000; if(ds_true>300){ double eo=fabs((clk_old-lo)/ds_true-1), en=fabs((clk_new-ln)/ds_true-1); if(eo>worst_o)worst_o=eo; if(en>worst_n)worst_n=en; }
      ls=xs; lo=clk_old; ln=clk_new; } }
  printf("%-34s true %6.1f mm | OLD %6.1f mm (%+5.1f%%, worst 30 ms window %3.0f%% off) | NEW %6.1f mm (%+5.1f%%, worst %3.0f%% off)\n",
    what,xs,clk_old/10000.0,(clk_old/10000.0/xs-1)*100,worst_o*100,clk_new/10000.0,(clk_new/10000.0/xs-1)*100,worst_n*100);
}
int main(void){
  run("straight, 95 mm/s",95,0,85);
  run("weave +-30 deg/s",95,30,85);
  run("weave +-55 deg/s (your log)",95,55,85);
  run("weave +-55, sensor really at 95 mm",95,55,95);
  run("weave +-55, sensor really at 75 mm",95,55,75);
  return 0; }
