#define s2t (15./bpm)
#define b2t (60./bpm)

const float pi=acos(-1.);
const float tau=pi*2.;
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
#define tri(p) (1.-4.*abs(fract(p)-0.5))
// midi 2 freq
#define p2f(i) (440.*exp2(((i)-69.)/12.))
#define f2p(i) (12.*(log2(i/440.))+69.)
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

vec2 cis(float t) {
  return vec2(cos(t), sin(t));
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
    vec2 wave = cheapfiltersaw(phase,mix(1.,.1,p3));
    sum += 2.0 * envg * wave;
  }

  return sum / float(count);
}
mat2 rot(float x) {
  vec2 v = cis(x);
  return mat2(v.x, v.y, -v.y, v.x);
}
mat3 orthBas(vec3 z) {
  z = normalize(z);
  vec3 x = normalize(cross(vec3(0, 1, 0), z));
  vec3 y = cross(z, x);
  return mat3(x, y, z);
}

vec3 cyclic(vec3 p, float pers, float lacu) {
  vec4 sum = vec4(0);
  mat3 rot = orthBas(vec3(2, -3, 1));

  for (int i = 0; i ++ < 5;) {
    p *= rot;
    p += sin(p.zxy);
    sum += vec4(cross(cos(p), sin(p.yzx)), 1);
    sum /= pers;
    p *= lacu;
  }

  return sum.xyz / sum.w;
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
// time:(a beat, a bar, sixteen bars, infinity)
vec2 mainAudio( vec4 time ) {  
  vec4 lt=time*bpm/60.0;
  ivec4 it=ivec4(floor(lt));
  vec2 dest = vec2( 0.0 );

  float s=.0;
  {// kick
    float t=time.y;
    vec4 seq=seq16(t,b2t*.25,0x8888);
    // dest+=exp(-seq.t*3.)*shotgun(200.*t,1.,.5,.5);
    dest+=kick(seq.t);
    // https://scrapbox.io/0b5vr/Zero_Crossing
    s=smoothstep(.0,.8*b2t,t)*smoothstep(.0,.001,seq.q);
  }
  {// hihat
    vec4 seq = swseq16(time.y,b2t*.25,0xffff);
    float t = seq.t;
    // https://scrapbox.io/0b5vr/fract_step_velocity
    float vel = fract(seq.s * 0.62+.71);
    float env = exp2(-exp2(6.0 - 1.0 * vel) * t);
    vec2 wave = shotgun(6000.*t,2.,.0,.5);
    dest += 0.2 * env * mix(0.2, 1.0, s) * tanh(8.0 * wave);
  }
  {
    float t=time.x;

    // https://scrapbox.io/0b5vr/SuperSaw
    // dest += exp(-3.*t)*
    //   vec2(cheapfiltersaw(t*p2f(60.+float(it.y)),.01),
    //   cheapfiltersaw(t*p2f(59.9+float(it.y)),.02)
    //   )*rot(pi*t*p2f(100.*p0));
    dest+=j_wouf(t)*vec2(cheapfiltersaw(t*p2f(60.+float(it.y)),.01),
      cheapfiltersaw(t*p2f(59.9+float(it.y)),.02)
      )*rot(pi*t*p2f(100.*p0))*.1;
  }
  
  
  dest=tanh(dest);
  return dest;
}
