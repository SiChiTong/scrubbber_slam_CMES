
//输入变量
attribute vec3  vPosition; 
attribute vec2  texcoord;

uniform  mat4       matModel;
uniform  mat4       matProject;
uniform  vec3       vecParam;

//输出变量
varying vec2 outTexcoord;
varying float  outselected;

void main()                                
{      
                                    
  gl_Position = matProject * matModel* vec4(vPosition, 1.0);
  outselected = vecParam.x;
  outTexcoord.x = texcoord.x;
  outTexcoord.y = texcoord.y;
}                                          
