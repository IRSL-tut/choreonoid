#version 330

in vec2 texCoord;
layout(location = 0) out vec4 fragColor;

uniform sampler2D textTexture;
uniform vec3 textColor;
uniform float opacity = 1.0;
uniform bool isPickingEnabled = false;
uniform bool isDepthPeelingEnabled = false;
uniform bool isDepthPeelingReversedDepth = false;
uniform sampler2D peeledDepthTexture;

void main()
{
    float coverage = texture(textTexture, texCoord).r;
    if(coverage <= 0.0){
        discard;
    }

    if(isPickingEnabled){
        fragColor = vec4(textColor, 1.0);
        return;
    }

    float alpha = coverage * opacity;
    if(alpha <= 0.0){
        discard;
    }

    if(isDepthPeelingEnabled){
        float peeledDepth = texelFetch(peeledDepthTexture, ivec2(gl_FragCoord.xy), 0).r;
        if(isDepthPeelingReversedDepth ?
           (gl_FragCoord.z >= peeledDepth) : (gl_FragCoord.z <= peeledDepth)){
            discard;
        }
    }

    fragColor = vec4(textColor, alpha);
}
