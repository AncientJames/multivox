#version 310 es

precision mediump int;
precision mediump float;

#ifdef VOXEL_565
precision mediump sampler3D;
uniform sampler3D u_volume;
#else
precision mediump isampler3D;
uniform isampler3D u_volume;
#endif

uniform int u_bpcmask;
uniform int u_dotlock;

in vec3 v_texcoord;
in vec2 v_dotcoord;
in vec3 v_bpcscale;

out vec4 ql_FragColor;

float dot2(vec2 v) {return dot(v, v);}

void main() {
    float rsq = dot2(fract(v_dotcoord)-vec2(0.5, 0.5));
    float lum = max(0.0, 0.5 - rsq * 2.0);

#ifndef LOW_QUALITY
    float dm = v_dotcoord.x + v_dotcoord.y;
    float dd = min(1.0, 0.125 / dot2(vec2(dFdx(dm), dFdy(dm))));
    lum = mix(0.125, lum, dd);
#endif

    vec3 texcoord = v_texcoord;
#ifndef LOW_QUALITY
    if (u_dotlock != 0) {
        texcoord.yz = (texcoord.yz - vec2(0.5, 0.5)) * ((floor(v_dotcoord.x) + 0.5) / v_dotcoord.x) + vec2(0.5, 0.5);
    }
#endif

#ifdef VOXEL_565
    vec3 colour = texture(u_volume, texcoord).rgb;
#else
    int pix = texture(u_volume, texcoord).r & u_bpcmask;
    vec3 colour = vec3(float(pix & 0xe0), float(pix & 0x1c), float(pix & 0x03)) * v_bpcscale;
#endif

    ql_FragColor.rgb = colour * lum;
    ql_FragColor.a = 1.0;
}
