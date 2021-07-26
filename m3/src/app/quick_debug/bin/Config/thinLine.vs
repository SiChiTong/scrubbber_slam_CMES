//输入变量
attribute vec4 vPosition; 
attribute vec3 vColor;     

uniform mat4  matModel;
uniform mat4  matProject;
uniform vec2  uWorld;

//输出变量
varying vec4  varColor;
varying vec2  varPos;

void main()                                
{                                          
    varColor = vec4(vColor, 1.0);
    gl_Position = matProject * matModel*  vPosition;

    varPos = (gl_Position.xy / gl_Position.w + 1.0) / 2.0 *  uWorld;
}                                          
