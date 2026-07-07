#version 330

/*
  Fragment shader to copy the depth values of the opaque scene into the
  depth buffer of the peeling framebuffer. This makes the ordinary depth
  test hide the transparent fragments behind the opaque objects in each
  peeling pass. The shader-based copy is used instead of glBlitFramebuffer
  because the formats of the two depth buffers are different.

  Both the sampler2D and sampler2DMS uniforms are defined and switched by
  the useMsaa flag in the same way as SolidPoint.frag.

  In the supersampled depth peeling mode, the peeling framebuffer is
  viewportScale times larger than the opaque scene, so the fragment
  coordinates are divided by the scale to sample the corresponding
  opaque depth values.
*/

uniform sampler2D depthTexture2D;
uniform sampler2DMS depthTextureMS;
uniform bool useMsaa;
uniform int viewportScale = 1;

void main()
{
    ivec2 p = ivec2(gl_FragCoord.xy) / viewportScale;
    if(useMsaa){
        // Sample 0 is used as the representative depth of the multisample opaque scene
        gl_FragDepth = texelFetch(depthTextureMS, p, 0).r;
    } else {
        gl_FragDepth = texelFetch(depthTexture2D, p, 0).r;
    }
}
