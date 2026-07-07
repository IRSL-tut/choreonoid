#version 330

/*
  Vertex shader for the full-screen passes used in the depth peeling.
  A single triangle covering the entire screen is generated only from
  gl_VertexID, so no vertex buffer is needed. Draw it with
  glDrawArrays(GL_TRIANGLES, 0, 3) and an empty VAO.
*/

void main()
{
    vec2 pos = vec2(float((gl_VertexID << 1) & 2), float(gl_VertexID & 2));
    gl_Position = vec4(pos * 2.0 - 1.0, 0.0, 1.0);
}
