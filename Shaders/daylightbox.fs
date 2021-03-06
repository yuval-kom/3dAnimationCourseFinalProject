#version 330 core
out vec4 FragColor;

in vec3 TexCoords;

uniform samplerCube daylightbox;

void main()
{    
    FragColor = texture(daylightbox, TexCoords);
}