/* Barcode pipeline model: IR spot + comparator (no hysteresis) -> 0.5 ms sampler (drv_track.c
   timer3_inthdr logic) -> 32-entry edge queue -> barcode task poll 5 ms -> REAL barcode_decode.c.
   Barcode per spec: 141 mm for 3 characters => narrow 3 mm, wide 9 mm. */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include "barcode_decode.h"
static double UNIT_MM=3.0;
#define RES 0.01              /* mm per profile cell */
#define MAXC 40000
static double prof[MAXC]; static int ncell;
static double blurred[MAXC];
static unsigned rng=1; static double urand(void){ rng=rng*1664525u+1013904223u; return (rng>>8)/16777216.0; }
static double gauss(void){ double a=urand()+1e-9,b=urand(); return sqrt(-2*log(a))*cos(6.2831853*b); }
static void build(int letter, double spot_mm){
  unsigned short pats[3]={0x094, barcode_letter_pattern((unsigned char)letter), 0x094};
  double x=20.0; int i,c,e; memset(prof,0,sizeof prof);
  for(c=0;c<3;c++){ for(e=0;e<9;e++){ double w=((pats[c]>>(8-e))&1)?3*UNIT_MM:UNIT_MM;
      if((e&1)==0){ for(i=(int)(x/RES); i<(int)((x+w)/RES); i++) prof[i]=1.0; } x+=w; }
    x+=UNIT_MM; }
  ncell=(int)((x+20.0)/RES);
  { int h=(int)(spot_mm/2/RES); for(i=0;i<ncell;i++){ double s=0; int n=0,k; for(k=-h;k<=h;k++){ int j=i+k; if(j>=0&&j<ncell){ s+=prof[j]; n++; } } blurred[i]=s/n; } }
}
/* sampler variants */
static int stable_n=2, integ_n=0;
typedef struct { unsigned q_level, samp_level, cand_n, cand_us; int acc; unsigned first_us; } samp_t;
static barcode_edge_queue_t q; static unsigned pushed, rawchg;
static void sampler(samp_t*s, unsigned level, unsigned now){
  if(integ_n>0){            /* proposed: saturating up/down counter = hysteresis in time */
    if(level){ if(s->acc<integ_n) s->acc++; } else { if(s->acc>0) s->acc--; }
    if(level!=s->samp_level){ if(!s->cand_n){ s->cand_us=now; s->cand_n=1; } } else s->cand_n=0;
    if((s->acc==integ_n && s->samp_level==0) || (s->acc==0 && s->samp_level==1)){
      s->samp_level=!s->samp_level; s->cand_n=0;
      if(s->samp_level!=s->q_level){ s->q_level=s->samp_level; if(barcode_eq_push(&q, now-(unsigned)(integ_n*250))) pushed++; }
    }
    return; }
  if(level==s->samp_level){ s->cand_n=0; return; }
  if(s->cand_n==0) s->cand_us=now;
  s->cand_n++;
  if(s->cand_n<(unsigned)stable_n) return;
  s->cand_n=0; s->samp_level=level;
  if(level!=s->q_level){ s->q_level=level; if(barcode_eq_push(&q,s->cand_us)) pushed++; }
}
/* one pass; returns 1 correct, 0 invalid/none, -1 WRONG letter */
static int pass(int letter,double v_mms,double spot,double thr,double noise,double ripple){
  samp_t s={0}; barcode_asm_t a; barcode_t res; barcode_edge_t e; unsigned char seen=0; int got=0, out=0;
  unsigned t; double x=0; unsigned prev=0;
  build(letter,spot); barcode_eq_init(&q); barcode_asm_init(&a); pushed=rawchg=0;
  for(t=0; ; t+=500){
    double v=v_mms*(1.0+ripple*sin(6.2831853*5.0*t/1e6));
    x+=v*0.0005; if(x/RES>=ncell-1) break;
    { double val=blurred[(int)(x/RES)]+noise*gauss(); unsigned level=(val>thr); if(level!=prev){prev=level;rawchg++;} sampler(&s,level,t+1000000u); }
    if((t%5000)==0){
      while(barcode_eq_pop(&q,&e)){ if(barcode_asm_edge_arrived(&a,e.t_us,&res)){ if(barcode_eq_overflow_delta(&q,&seen)) res=barcode_invalid(); if(barcode_is_valid(res)){ got=1; out=(barcode_get_char(res)=='A'+letter)?1:-1; } } }
      if(barcode_asm_time_passed(&a,t+1000000u,&res)){ if(barcode_is_valid(res)&&!got){ got=1; out=(barcode_get_char(res)=='A'+letter)?1:-1; } }
    }
  }
  return out;
}
static void sweep(const char*label,double v,double spot,double thr,double noise,double ripple){
  int ok=0,bad=0,n=0,l,k; unsigned long edges=0;
  for(l=0;l<26;l++) for(k=0;k<8;k++){ int r=pass(l,v,spot,thr,noise,ripple); n++; if(r>0)ok++; if(r<0)bad++; edges+=pushed; }
  printf("%-40s v=%3.0f mm/s spot=%.1f thr=%.2f noise=%.2f ripple=%.0f%% : %3d/%d read, %d wrong, avg %.0f edges queued (30 = clean)\n",label,v,spot,thr,noise,ripple*100,ok,n,bad,(double)edges/n);
}
static void row(double unit,double v,double spot,double thr,double noise){
  int ok=0,bad=0,n=0,l,k; unsigned long edges=0; UNIT_MM=unit;
  for(l=0;l<26;l++) for(k=0;k<4;k++){ int r=pass(l,v,spot,thr,noise,0); n++; if(r>0)ok++; if(r<0)bad++; edges+=pushed; }
  printf("narrow %.1f mm (barcode %3.0f mm) spot %.0f mm thr %.2f noise %.2f v %3.0f: %3d/%d read, %d wrong, %2.0f edges\n",unit,47*unit,spot,thr,noise,v,ok,n,bad,(double)edges/n);
}
int main(int argc,char**argv){
  double units[]={3.0,2.5,2.0,1.5}; double spots[]={2,3,4}; double thrs[]={0.5,0.4,0.3,0.2}; int i,j,k;
  if(argc>1) integ_n=atoi(argv[1]);
  for(j=0;j<3;j++) for(k=0;k<4;k++) for(i=0;i<4;i++) row(units[i],65,spots[j],thrs[k],0);
  for(i=0;i<4;i++) row(units[i],65,2,0.5,0.06);
  for(i=0;i<4;i++) row(units[i],200,2,0.4,0);
  return 0; }
