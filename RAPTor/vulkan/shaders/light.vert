#version 450
#extension GL_ARB_separate_shader_objects : enable

layout(set = 0, binding = 0) uniform UniformBufferObject
{
    mat4 view;
    mat4 proj;
} ubo;

layout(location = 0) in vec3 inPosition;
layout(location = 3) in mat4 modelTransform;


void main()
{
    gl_Position = ubo.proj * ubo.view * modelTransform * vec4(inPosition, 1.0);
}
