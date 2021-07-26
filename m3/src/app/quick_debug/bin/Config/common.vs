
//输入变量
attribute vec4 vPosition; 
attribute vec3 vColor;       

uniform mat4  matModel;
uniform mat4  matProject;

//输出变量
varying vec3  varColor;

void main()                                
{                                          
	varColor = vColor;
  gl_Position = matProject * matModel* vPosition;
}                                          
