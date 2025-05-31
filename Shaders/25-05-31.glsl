#define bl (60./bpm)

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



// time:(a beat, a bar, sixteen bars, infinity)
vec2 mainAudio( vec4 time ) {
  
  vec4 lt=time*bpm/60.0;
  ivec4 it=ivec4(floor(lt));
  vec2 dest = vec2( 0.0 );

  float s=mix(1.,.3,smoothstep(bl,.0,time.x)*linearstep(.0,.001,time.x));

  vec2 o=vec2(0);
  float duck=.0;
  {
    float t=time.x;
    float a=tan((-lt.x+0.5)*pi);a=pow(abs(a),a);
    o=exp(-t*3.)*spray(t,40.,.4,.0,.001,8)*.8;
    o+=kick(time.x)*.6;
    dest+=o*p5;
    duck=exp(-t*3.);
  }

  {
    // hi hat m7使ったけど意味ない？かもしれない
    float n = float(int(lt.x*4.)!=0);n=1.;
    float t = mod(time.x,bl*.25);
    float nt = t+(hash(vec3(time.x,it.w,2)).x-.5)*n/pi/220.*4.;
    float b = 220.0; // Base frequency (A3)
    // Minor 7th chord intervals (semitones from root)
    float f1 = b*s2r(0.0);// Root note 基音 (220Hz = A3)
    float f2 = b*s2r(3.0);// Minor third 短3度 (C)
    float f3 = b*s2r(7.0);// Perfect fifth 完全5度 (E)
    float f4 = b*s2r(10.0);// Minor seventh 短7度 (G)
    // Generate oscillators for each note
    float o1 = osc(f1,nt);
    float o2 = osc(f2,nt);
    float o3 = osc(f3,nt);
    float o4 = osc(f4,nt);
    // Mix the oscillators
    vec2 mix = vec2(o1 +o4 ,o2 +o3)/2.0*.4;
    dest+=exp(-t*130.)*mix*p4;
  }

  {
    // float t, float freq, float spread, float seed, float interval, int count
    float t=time.x;
    // float notes[4] = ;
    float tr=fract(lt.x*4.);
    float b=floor(lt.x*4.);
    float a=exp(-tr*1.);
    int n=1+int(b);
    int span=8;
    vec3 h1=hash(vec3(it.w/span,it.w%2,2)),h2=hash(vec3(it.w/span,it.w%2,1));
    vec4 tp=round((vec4(h1,h2.x)-.5)*8.)*4.;
    vec4 cho=vec4[](vec4(0,3,7,10),vec4(3,7,10,14),vec4(7,10,14,17),vec4(7,10,14,22))[(it.w/span)%4];
    float ga=mix(2.,16.,p2);
    float base = mix(42.,52.,p0);
    rep(i,n){
      float m=base+cho[i]+tp[int(b)]-duck;
      float sh=float((it.w/8)%2)+float(i)*16.;
      // float sh=float(i);
      // dest+=a*spray(t,m2f(m),p1*2.,sh,.04,16)*mix(vec2(1),vec2(.3),duck)*1.5;
      // if((it.w/4)%8==7)dest+=a*spray(t,m2f(m),p1,sh,.04,16)*mix(vec2(1),vec2(.3),duck)*1.5;
      // else dest+=a*spray(t,m2f(m-16.),ga,sh,.04,4)*o*1.5;
      // else dest+=a*spray(t,m2f(m-16.),ga,sh,.04,4)*mix(vec2(.6),vec2(0),o);
      dest+=a*spray(t,m2f(m),p1,sh,.04,16)*mix(vec2(1),vec2(.3),duck)*1.5;
      dest+=a*spray(t,m2f(m-16.),ga,sh,.04,4)*o*1.5;
      // dest+=a*spray(t,m2f(m-16.),ga,sh,.04,4)*mix(vec2(.6),vec2(0),o);
    }
  }
  
  dest=tanh(dest);
  return dest;
}
