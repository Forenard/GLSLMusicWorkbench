// dubっぽいなにか bpm140
#define BPM bpm

#define PI 3.14159265359
#define TAU 6.28318530718
#define rep(i,n) for(int i=0;i<n;i++)
#define sat(x) clamp(x,.0,1.)
uniform vec4 param_knob0;
uniform vec4 param_knob1;
#define p0 paramFetch(param_knob0)
#define p1 paramFetch(param_knob1)

float kick( float t ) {
  if ( t < 0.0 ) { return 0.0; }

  float attack = 4.0;

  return exp( -4.0 * t ) * sin( TAU * (
    50.0 * t - attack * ( exp( -40.0 * t ) + exp( -200.0 * t ) )
  ) );
}

vec3 hash(vec3 x)
{
  uvec3 v=floatBitsToUint(x+vec3(.1,.2,.3));
  uint k=0xf920a81fu;
  v=((v>>8u)^v.yzx)*k;v=((v>>8u)^v.yzx)*k;v=((v>>8u)^v.yzx)*k;
  return vec3(v)/vec3(-1u);
}
vec3 cyc(vec3 x,float q)
{
  vec4 v = vec4(0);
  rep(i,16)
  {
    x += sin(x.yzx);
    v = v * q + vec4(cross(cos(x),sin(x.zxy)),1);
    x *= q;
  }
  return v.xyz / v.w;
}


// time:(a beat, a bar, sixteen bars, infinity)
vec2 mainAudio( vec4 time ) {
  vec2 dest = vec2( 0.0 );

  // kick
  float s=.0;
  {
    float kk=.6*kick(time.x);
    dest += kk;
    s=exp(-abs(kk)*16.*p1);
  }
  
  // セラミック皿の衝突
  {
    float[] freq = float[](3164.,10255.,18204.);// 固有振動数 (Hz)
    float[] damp = float[](.1,.15,.2);// 減衰率 (1/s)
    float[] amp  = float[](.33,.33,.33);// 振幅（仮に等分配）
    for(int i=0;i<3;i++){
      float f=freq[i],d=damp[i],a=amp[i];
      d*=4.;a*=.3;
      dest+=a*exp(-d*time.z)*sin(TAU*f*time.z);
      dest+=a*exp(-d*time.z)*sin(TAU*(f+time.z)*time.z);
    }
    dest+=(hash(time.xyw).xy*2.-1.)*exp(-time.z)*.1;
  }

  // キュ
  {
    float t=pow(sat(time.z*bpm/60./64.),8.);
    vec3 h1=hash(vec3(floor(time.w*bpm/60.),2,3));
    vec2 f=cyc(vec3(time.z,h1.x,2),TAU).xy*4000.*t;
    dest+=.3*exp(-2.*time.x)*sin(TAU*f*time.x)*s*t;
  }

  // 何かの音
  {
    int bt=int(floor(time.w*bpm/60.));
    vec3 chords[4] = vec3[4](
      vec3(261.63, 329.63, 392.00), // C
      vec3(392.00, 493.88, 587.33), // G
      vec3(220.00, 261.63, 329.63), // A
      vec3(349.23, 440.00, 523.25)  // F
    );
    float arp=.0;
    float dub=float(bt%4!=3);
    vec3 h0=hash(vec3(floor(time.w*bpm/60.),1,2));
    rep(i,3){
      //float f=440.+40.*(floor(h0.x*16.)+float(i));
      float t=max(.0,time.x-float(i)*60./bpm/3.);
      float f = chords[int(time.y*bpm/60.)%4][i]-s*20.*p0*floor(h0.y*4.+4.)*dub;
      arp+=exp(-3.*t)*sin(TAU*f*t);
    }
    dest+=arp*s*(.5-dub*.2);
  }

  dest = tanh(dest);
  return dest;
}
