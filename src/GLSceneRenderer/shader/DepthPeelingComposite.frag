#version 330

/*
  Fragment shader to composite a peeled layer texture over the main
  framebuffer. The layers are composited in back-to-front order.

  The output is in the premultiplied alpha form and must be blended with
  glBlendFunc(GL_ONE, GL_ONE_MINUS_SRC_ALPHA). This form allows the
  supersampled layer texels to be averaged correctly in the downsampling;
  when viewportScale is 1, the result is identical to the standard alpha
  blending. Pixels not covered by the layer have zero alpha and do not
  change the destination.
*/

uniform sampler2D layerTexture;
uniform int viewportScale = 1;

layout(location = 0) out vec4 color4;

void main()
{
    ivec2 p = ivec2(gl_FragCoord.xy) * viewportScale;
    vec4 sum = vec4(0.0);
    for(int i=0; i < viewportScale; ++i){
        for(int j=0; j < viewportScale; ++j){
            vec4 c = texelFetch(layerTexture, p + ivec2(i, j), 0);
            sum += vec4(c.rgb * c.a, c.a);
        }
    }
    color4 = sum / float(viewportScale * viewportScale);
}
