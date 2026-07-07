#version 330

/*
  Fragment shader to composite a peeled layer texture over the main
  framebuffer. The layers are composited in back-to-front order with
  the standard alpha blending (GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA).
  Pixels not covered by the layer have zero alpha and do not change
  the destination.
*/

uniform sampler2D layerTexture;

layout(location = 0) out vec4 color4;

void main()
{
    color4 = texelFetch(layerTexture, ivec2(gl_FragCoord.xy), 0);
}
