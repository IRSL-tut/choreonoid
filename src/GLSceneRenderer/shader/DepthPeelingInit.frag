#version 330

/*
  Fragment shader to copy the depth values of the opaque scene into the
  depth buffer of the peeling framebuffer. This makes the ordinary depth
  test hide the transparent fragments behind the opaque objects in each
  peeling pass. The shader-based copy is used instead of glBlitFramebuffer
  because the formats of the two depth buffers are different.

  Both the sampler2D and sampler2DMS uniforms are defined and switched by
  the useMsaa flag in the same way as SolidPoint.frag.
*/

uniform sampler2D depthTexture2D;
uniform sampler2DMS depthTextureMS;
uniform bool useMsaa;

void main()
{
    if(useMsaa){
        // Sample 0 is used as the representative depth of the multisample opaque scene
        gl_FragDepth = texelFetch(depthTextureMS, ivec2(gl_FragCoord.xy), 0).r;
    } else {
        gl_FragDepth = texelFetch(depthTexture2D, ivec2(gl_FragCoord.xy), 0).r;
    }
}
