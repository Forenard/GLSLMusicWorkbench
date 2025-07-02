#define bl (60./bpm)
#define s2t (15./bpm)
#define b2t (60./bpm)

#define pi 3.14159265359
#define tau 6.28318530718
#define rep(i,n) for(int i=0;i<n;i++)
#define sat(x) clamp(x,.0,1.)
#define linearstep(a,b,x) sat(((x)-(a))/((b)-(a)))
uniform vec4 param_knob0;
uniform vec4 param_knob1;
uniform vec4 param_knob2;
uniform vec4 param_knob3;
uniform vec4 param_knob4;
uniform vec4 param_knob5;
uniform vec4 param_knob6;
uniform vec4 param_knob7;
#define p0 paramFetch(param_knob0)
#define p1 paramFetch(param_knob1)
#define p2 paramFetch(param_knob2)
#define p3 paramFetch(param_knob3)
#define p4 paramFetch(param_knob4)
#define p5 paramFetch(param_knob5)
#define p6 paramFetch(param_knob6)
#define p7 paramFetch(param_knob7)

// sine wave oscillator
#define osc(_x,_t) sin(tau*(_x)*(_t))
// Convert semitone offset to frequency ratio
#define s2r(_s) pow(2.0,(_s)/12.0)
// midi 2 freq
float m2f(float m) {
    return 453.0 * exp2((m - 69.0) / 12.0);
}
// ADSR envelope (simplified)
float adsr(float time, float attack, float decay, float sustain, float release, float noteLength) {
  if (time < attack) {
    return time / attack;
  } else if (time < attack + decay) {
    return 1.0 - (1.0 - sustain) * (time - attack) / decay;
  } else if (time < noteLength - release) {
    return sustain;
  } else if (time < noteLength) {
    return sustain * (noteLength - time) / release;
  }
  return 0.0;
}
vec3 hash(vec3 x)
{
  uvec3 v=floatBitsToUint(x+vec3(.1,.2,.3));
  uint k=0xf920a81fu;
  v=((v>>8u)^v.yzx)*k;v=((v>>8u)^v.yzx)*k;v=((v>>8u)^v.yzx)*k;
  return vec3(v)/vec3(-1u);
}
uvec3 pcg3d( uvec3 v ) {
  v = v * 1145141919u + 1919810u;
  v.x += v.y * v.z;
  v.y += v.z * v.x;
  v.z += v.x * v.y;
  v ^= v >> 16u;
  v.x += v.y * v.z;
  v.y += v.z * v.x;
  v.z += v.x * v.y;
  return v;
}

vec3 pcg3df( vec3 v ) {
  uvec3 r = pcg3d( floatBitsToUint( v ) );
  return vec3( r ) / float( 0xffffffffu );
}

vec2 cis(float t) {
  return vec2(cos(t), sin(t));
}
mat2 rot(float x) {
  vec2 v = cis(x);
  return mat2(v.x, v.y, -v.y, v.x);
}
vec2 boxMuller(vec2 xi) {
  float r = sqrt(-2.0 * log(xi.x));
  float t = xi.y;
  return r * cis(tau * t);
}
float cheapfiltersaw(float phase,float k){
  float wave=mod(phase,1.);
  float c=.5+.5*cos(pi*sat(wave/k));
  return (wave+c)*2.-1.-k;
}
vec2 cheapfiltersaw(vec2 phase,float k){
  vec2 wave=mod(phase,1.);
  vec2 c=.5+.5*cos(pi*sat(wave/k));
  return (wave+c)*2.-1.-k;
}
vec2 spray(float t, float freq, float spread, float seed, float interval, int count) {
  float grainLength = float(count) * interval;

  vec2 sum = vec2(0.0);
  rep(i, count) {
    float fi = float(i);

    float off = -interval * fi;
    float tg = mod(t + off, grainLength);
    float prog = tg / grainLength;

    vec3 dice = hash(vec3(i, floor((t + off) / grainLength), seed));
    vec2 dicen = boxMuller(dice.xy);

    float envg = smoothstep(0.0, 0.5, prog) * smoothstep(1.0, 0.5, prog);
    // float envg  = smoothstep(0.0, 0.1, prog) * smoothstep(1.0, 0.1, prog);

    vec2 phase = vec2(freq * t);
    phase *= exp2(spread * dicen.xy);
    phase += dice.xy;

    // vec2 wave = sin(tau * phase);
    // vec2 wave = cheapfiltersaw(phase,pi/2.);
    // vec2 wave = asin(sin(tau * phase))/pi*2.;
    // vec2 wave = (fract(phase)*2.-1.)*.6;
    // vec2 wave = mix(sin(tau * phase),cheapfiltersaw(phase,mix(.1,1.,p3)),p2);
    vec2 wave = cheapfiltersaw(phase,mix(1.,.1,1.));
    sum += 2.0 * envg * wave;
  }

  return sum / float(count);
}
/*
float kick(float t) {
  float decay = exp(-5.0 * t);
  float phase = 50.0 * t;
  phase -= 3.0 * exp(-40.0 * t);
  return decay * sin(tau * phase + (hash(vec3(0.004 * t,2,1)).x-.5)*.1);
}
*/
#define fold(_x) (1.0 - 4.0 * abs(fract((_x)/ 4.0 + 0.25) - 0.5))
 
vec2 kick(float t) {
  float decay = exp(-5.0 * t);
  float phase = 40.0 * t;
  phase -= 6.0 * exp(-40.0 * t);
  float x=2. * decay * sin(tau * phase);
  return vec2(fold(x));
  // return fold(spray(x,4.,.4,.0,.005,64));
}

float qtree(float t,float seed){
  float th=.75;
  int n=2;
  float b=t/b2t,sp=.5;
  rep(i,n){
    vec3 h=hash(vec3(floor(b/sp),seed,1.4));
    if(th<h.x)sp/=3.;
  }
  b=mod(b,sp);
  t=b*b2t;
  return t;
}

const float swing = .66;
float t2sSwing(float t, float tps) {
  float st = t / tps;
  return 2.0 * floor(st / 2.0) + step(swing, fract(0.5 * st));
}
float s2tSwing(float st, float tps) {
  return 2.0 * tps * (floor(st / 2.0) + swing * mod(st, 2.0));
}
// https://scrapbox.io/0b5vr/Sinewave_Shotgun
vec2 shotgun(float t, float spread, float snap, float fm) {
  vec2 sum = vec2(0.0);

  rep(i, 64) {
    vec3 dice = hash(vec3(i + 1));

    vec2 partial = exp2(spread * dice.xy);
    partial = mix(partial, floor(partial + 0.5), snap);
    // partial=floor(partial + 0.5);

    sum += sin(tau * t * partial + fm * sin(tau * t * partial));
  }

  return sum / 64.0;
}
// swing
// https://scrapbox.io/0b5vr/Swing
// https://scrapbox.io/0b5vr/16bit_Int_Step_Sequence
vec4 swseq16(float t, float tps, int seq) {
   t = mod(t, 16.0 * tps);
   int sti = int(t2sSwing(t, tps));
   int rotated = ((seq >> (15 - sti)) | (seq << (sti + 1))) & 0xffff;
 
   float prevStepBehind = log2(float(rotated & -rotated));
   float prevStep = float(sti) - prevStepBehind;
   float prevTime = s2tSwing(prevStep, tps);
   float nextStepForward = 16.0 - floor(log2(float(rotated)));
   float nextStep = float(sti) + nextStepForward;
   float nextTime = s2tSwing(nextStep, tps);
 
   return vec4(
     prevStep,
     t - prevTime,
     nextStep,
     nextTime - t
   );
 }
float j_wouf(float x) {
  return 1.0 / (1.0 + exp(-50.0 * (x - 0.15))) * 1.0 / (1.0 + exp(-30.0 * (0.8 - x))) / (1.0 + exp(-6.0 * (0.9 - x)));
}

/*
0 ....    1 ...x    2 ..x.    3 ..xx
4 .x..    5 .x.x    6 .xx.    7 .xxx
8 x...    9 x..x    a x.x.    b x.xx
c xx..    d xx.x    e xxx.    f xxxx
s 現在trigger中のstep番号
t に現在trigger中の経過秒
q に次のtriggerまでの秒
https://scrapbox.io/0b5vr/16bit_Int_Step_Sequence
*/
vec4 seq16(float t, float tps, int seq) {
   t = mod(t, 16.0 * tps);
   int sti = int(t / tps) & 15;
   int rotated = ((seq >> (15 - sti)) | (seq << (sti + 1))) & 0xffff;
 
   float prevStepBehind = log2(float(rotated & -rotated));
   float prevStep = float(sti) - prevStepBehind;
   float prevTime = tps * prevStep;
   float nextStepForward = 16.0 - floor(log2(float(rotated)));
   float nextStep = float(sti) + nextStepForward;
   float nextTime = tps * nextStep;
 
   return vec4(
     prevStep,
     t - prevTime,
     nextStep,
     nextTime - t
   );
 }

// time:(a beat, a bar, sixteen bars, infinity)
vec2 mainAudio( vec4 time ) {
  
  vec4 lt=time*bpm/60.0;
  ivec4 it=ivec4(floor(lt));
  vec2 dest = vec2( 0.0 );

  vec2 o=vec2(0);
  float s=.0;
  {
    vec4 seq=seq16(time.y,s2t,0x8888);
    float t=seq.t;
    // https://scrapbox.io/0b5vr/Zero_Crossing
    
    float a=tan((-lt.x+0.5)*pi);a=pow(abs(a),a);
    o=exp(-t*3.)*spray(t,40.,.4,.0,.001,8);
    o+=kick(t);
    dest+=o*rot(t*50.)*.5;
    s=smoothstep(.0,.8*b2t,t)*smoothstep(.0,.001,seq.q);
  }

  {
    vec4 seq=seq16(time.y,s2t,0xffff);
    float t=seq.t;
    float sp=seq.t+seq.q;
    vec3 h=hash(vec3(seq.s,it.w/16,1.4));
    if(h.x<.2)t=mod(t,sp/2.);
    float vel = fract(seq.s*.61+.59);
    float env = exp2(-exp2(mix(4.,8.,vel)) * t);
    vec2 wave = shotgun(8000.*t,5.,.1,.5);
    dest += 0.2 * env * mix(0.2, 1.0, s) * tanh(8.0 * wave);
  }

  {
    vec4 seq=seq16(time.y,s2t,0xffff);
    float t=seq.t;
    float b=floor(lt.x*4.);
    float a=exp(-t*1.);
    int n=1+int(b);
    int span=16;
    vec3 h1=hash(vec3(it.w/span,it.w%2,2)),h2=hash(vec3(it.w/span,it.w%2,1));
    vec4 tp=round((vec4(h1,h2.x)-.5)*8.)*4.;
    vec4 chos[] = vec4[](vec4(0,3,7,10),vec4(3,7,10,14),vec4(7,10,14,17),vec4(7,0,14,22));
    vec4 cho=chos[(it.w/span)%4];
    float ga=8.;
    float base = 46.;
    vec2 w=vec2(0);
    rep(i,n){
      float m=base+cho[i]+tp[n-1]-s+float((it.w/8)%2==1)*4.;
      float sh=float(i)*16.;
      // float sh=float(i);
      // dest+=a*spray(t,m2f(m),p1*2.,sh,.04,16)*mix(vec2(1),vec2(.3),duck)*1.5;
      // if((it.w/4)%8==7)dest+=a*spray(t,m2f(m),p1,sh,.04,16)*mix(vec2(1),vec2(.3),duck)*1.5;
      // else dest+=a*spray(t,m2f(m-16.),ga,sh,.04,4)*o*1.5;
      // else dest+=a*spray(t,m2f(m-16.),ga,sh,.04,4)*mix(vec2(.6),vec2(0),o);
      w+=a*spray(t,m2f(m),.0,sh,.04,16)*mix(vec2(1),vec2(.3),s)*1.5;
      // w+=a*spray(t,m2f(m-16.),ga,sh,.04,4)*o*1.5;
      // dest+=a*spray(t,m2f(m-16.),ga,sh,.04,4)*mix(vec2(.6),vec2(0),o);
    }
    dest+=w*.5*mix(0.0,1.0, s);
      //*j_wouf(t);
  }
  dest=tanh(dest);
  return dest;
}
